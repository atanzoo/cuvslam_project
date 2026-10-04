#!/usr/bin/env python3
"""Build the single-file bilingual MPPI reading annotation."""

from __future__ import annotations

import base64
import html
import json
import re
from dataclasses import dataclass, asdict
from pathlib import Path
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[3]
TMP = REPO_ROOT / "tmp" / "pdfs"
SOURCE_PDF = (
    REPO_ROOT
    / "research"
    / "papers"
    / "navigation"
    / "mppi"
    / "01_information_theoretic_mppi_2017.pdf"
)
BBOX_PATH = TMP / "mppi_bbox_full.html"
UNITS_PATH = TMP / "mppi_units.json"
TRANSLATIONS_PATH = TMP / "mppi_translations.json"
OUTPUT_PATH = REPO_ROOT / "output" / "html" / "mppi_2017_bilingual_annotation_zh_tw.html"
RENDER_DIR = TMP / "mppi_2017_render"


SECTION_TITLES = {
    "abstract": ("摘要", "研究問題、方法與主要比較的濃縮陳述"),
    "introduction": ("I. 緒論", "從自駕控制缺口推進至 IT-MPC 的核心主張"),
    "preliminaries": ("II. 預備知識", "建立動力系統、成本函數與資訊理論記號"),
    "it-mpc": ("III. 資訊理論模型預測控制", "由自由能界限推導取樣式控制更新律"),
    "stochastic-control": ("IV. 與隨機最佳控制的關係", "證明資訊理論觀點與路徑積分控制的連結"),
    "related-work": ("V. 取樣式控制相關研究", "以交叉熵法作為理論與實驗比較基線"),
    "experimental-setup": ("VI. 實驗設置", "交代車輛、模型、成本函數與控制器參數"),
    "results": ("VII. 結果", "以成功率、圈速、擾動與模型誤差檢驗主張"),
    "discussion": ("VIII. 討論", "界定方法優勢、限制與可推廣範圍"),
    "references": ("參考文獻", "原文引文資料"),
}


SECTION_PATTERNS = [
    ("introduction", r"^I\.\s+I?\s*N\s*T\s*R\s*O\s*D\s*U\s*C\s*T\s*I\s*O\s*N$"),
    ("preliminaries", r"^II\.\s+P\s*R\s*E\s*L\s*I\s*M\s*I\s*N\s*A\s*R\s*I\s*E\s*S$"),
    ("it-mpc", r"^III\.\s+I\s*N\s*F\s*O\s*R\s*M\s*A\s*T\s*I\s*O\s*N"),
    ("stochastic-control", r"^IV\.\s+R\s*E\s*L\s*A\s*T\s*I\s*O\s*N"),
    ("related-work", r"^V\.\s+R\s*E\s*L\s*A\s*T\s*E\s*D"),
    ("experimental-setup", r"^VI\.\s+E\s*X\s*P\s*E\s*R\s*I\s*M\s*E\s*N\s*T\s*A\s*L"),
    ("results", r"^VII\.\s+R\s*E\s*S\s*U\s*L\s*T\s*S$"),
    ("discussion", r"^VIII\.\s+D\s*I\s*S\s*C\s*U\s*S\s*S\s*I\s*O\s*N$"),
    ("references", r"^R\s*E\s*F\s*E\s*R\s*E\s*N\s*C\s*E\s*S$"),
]


@dataclass
class Unit:
    page: int
    section: str
    kind: str
    english: str
    source_index: int
    translation_index: int | None = None


def normalize_words(text: str) -> str:
    text = re.sub(r"(\w)-\s+(\w)", r"\1\2", text)
    text = re.sub(r"\s+([,.;:?!\)])", r"\1", text)
    text = re.sub(r"([\(\[])\s+", r"\1", text)
    text = re.sub(r"\s+", " ", text).strip()
    return text


def read_blocks() -> list[tuple[int, float, float, str]]:
    raw = BBOX_PATH.read_bytes()
    raw = bytes(b for b in raw if b in (9, 10, 13) or b >= 32)
    root = ET.fromstring(raw)
    ns = {"x": "http://www.w3.org/1999/xhtml"}
    blocks: list[tuple[int, float, float, str]] = []
    for page_number, page in enumerate(root.findall(".//x:page", ns), start=1):
        page_blocks = []
        for block in page.findall(".//x:block", ns):
            words = [word.text or "" for word in block.findall(".//x:word", ns)]
            text = normalize_words(" ".join(words))
            if not text:
                continue
            x = float(block.attrib["xMin"])
            y = float(block.attrib["yMin"])
            if y < 42 and (text == "MANUSCRIPT" or text.isdigit()):
                continue
            if text.startswith("arXiv:1707.02342"):
                continue
            column = 0 if x < 305 else 1
            page_blocks.append((column, y, x, text))
        page_blocks.sort(key=lambda row: (row[0], row[1], row[2]))
        blocks.extend((page_number, x, y, text) for _, y, x, text in page_blocks)
    return blocks


def detect_section(text: str, current: str) -> str:
    for section, pattern in SECTION_PATTERNS:
        if re.match(pattern, text, re.I):
            return section
    return current


def classify(text: str) -> str:
    if any(re.match(pattern, text, re.I) for _, pattern in SECTION_PATTERNS):
        return "heading"
    if re.match(r"^[A-E]\.\s+", text):
        return "subheading"
    if text.startswith(("Fig.", "TABLE ", "Algorithm ")):
        return "caption"
    if re.match(r"^\[[0-9]+\]", text):
        return "reference"
    word_count = len(text.split())
    alpha_count = len(re.findall(r"[A-Za-z]", text))
    if word_count < 18 and alpha_count < 35:
        return "equation"
    return "paragraph"


def should_skip(text: str) -> bool:
    skip_starts = (
        "Grady Williams, Brian Goldfain",
        "Paul Drews is with",
        "Evangelos A. Theodorou is with",
        "Manuscript received",
    )
    if text.startswith(skip_starts):
        return True
    if text.startswith("He has authored more than"):
        return True
    return False


def extract_units() -> list[Unit]:
    units: list[Unit] = []
    current_section = "abstract"
    for source_index, (page, _x, _y, text) in enumerate(read_blocks()):
        if should_skip(text):
            continue
        if page == 20 and not text.startswith("["):
            continue
        if text.startswith("Information Theoretic Model Predictive Control:"):
            continue
        if text.startswith("Grady Williams, Paul Drews,"):
            continue
        next_section = detect_section(text, current_section)
        kind = classify(text)
        if kind == "heading":
            current_section = next_section
            continue
        if text.startswith("Index Terms"):
            kind = "keywords"
        if page >= 20 and current_section == "references" and "received the" in text:
            continue
        units.append(
            Unit(
                page=page,
                section=current_section,
                kind=kind,
                english=text,
                source_index=source_index,
            )
        )

    # Join obvious continuations split by page or column boundaries.
    merged: list[Unit] = []
    for unit in units:
        continuation = bool(
            re.match(
                r"^(?:and|or|but|where|which|that|with|in which|robotics[.]|level)(?:\b|\s)",
                unit.english,
                re.I,
            )
        )
        if continuation and unit.kind == "paragraph":
            found_previous = False
            for previous in reversed(merged):
                if previous.section != unit.section:
                    break
                if previous.kind == "paragraph":
                    previous.english = normalize_words(previous.english + " " + unit.english)
                    found_previous = True
                    break
            if found_previous:
                continue
        merged.append(unit)

    translation_index = 0
    for unit in merged:
        if unit.kind not in {"equation", "reference"}:
            unit.translation_index = translation_index
            translation_index += 1
    return merged


def write_translation_input(units: list[Unit]) -> None:
    source = [
        unit.english
        for unit in units
        if unit.translation_index is not None
    ]
    UNITS_PATH.write_text(
        json.dumps([asdict(unit) for unit in units], ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    (TMP / "mppi_translation_input.json").write_text(
        json.dumps(source, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


TRANSLATION_FIXES = {
    "最優": "最佳",
    "隨機最優控制": "隨機最佳控制",
    "最優控制": "最佳控制",
    "最優分佈": "最佳分佈",
    "最優軌跡": "最佳軌跡",
    "最優係數": "最佳係數",
    "推匯出": "推導出",
    "抽樣": "取樣",
    "汙垢測試軌道": "土質測試賽道",
    "易操作性": "可處理性",
    "本地方法": "局部方法",
    "線下": "離線",
    "制定政策": "建立策略",
    "無衍生物": "無導數",
    "後視界": "滾動時域",
    "控制制度": "控制工況",
    "自動駕駛汽車": "自動駕駛車輛",
    "控制架構和寫程式": "控制架構與程式設計",
    "交叉熵方法": "交叉熵法",
    "圖。": "圖",
    "反向溫度": "逆溫度",
    "蒙特卡洛": "蒙地卡羅",
    "KL-分歧": "KL 散度",
    "基分佈": "基準分佈",
    "基本措施": "基準測度",
    "全域性": "全域",
    "區域性": "局部",
    "高斯過程迴歸": "高斯程序迴歸",
    "主動自動駕駛": "高動態自動駕駛",
    "激進的自動駕駛": "高動態自動駕駛",
    "激進駕駛": "高動態駕駛",
    "積極駕駛": "高動態駕駛",
    "非平凡": "並不容易",
}


SECTION_ARGUMENTS = {
    "abstract": {
        "task": "用最短篇幅提出方法、應用場景與比較對象。",
        "chain": "資訊理論推導 → 取樣式 MPC → 高動態自駕 → 與 CEM 比較。",
        "focus": "摘要只宣告有效性；真正的證明分別落在第三、四節與第七節。",
    },
    "introduction": {
        "task": "把高動態駕駛界定成既需要完整動力學、又難以即時求解的控制問題。",
        "chain": "分層規劃/追蹤的限制 → 傳統最佳控制不可即時計算 → GPU 取樣使 IT-MPC 可行。",
        "focus": "核心主張不是 MPPI 永遠優於分層架構，而是在非凸、非線性與硬成本下提供可即時運行的替代方案。",
    },
    "preliminaries": {
        "task": "定義後續推導所需的系統、控制分佈、成本與資訊量。",
        "chain": "隨機控制輸入 → 軌跡成本 → 自由能 → KL 散度。",
        "focus": "高斯控制雜訊與有限時域是推導契約；應與未來機器人的控制週期和動力模型逐項核對。",
    },
    "it-mpc": {
        "task": "從自由能界限得到最佳控制分佈，再轉化成可由取樣近似的控制更新律。",
        "chain": "Jensen 不等式 → 自由能下界 → 最佳分佈 Q* → 重要性取樣 → 即時 MPC 演算法。",
        "focus": "這是全文的理論核心；λ、Σ、成本尺度與取樣分佈共同決定控制器的探索與選擇性。",
    },
    "stochastic-control": {
        "task": "證明資訊理論推導並非孤立技巧，而與經典路徑積分隨機控制一致。",
        "chain": "HJB 變換 → 線性化條件 → Feynman-Kac 表示 → 與資訊理論控制律對齊。",
        "focus": "精確等價依賴控制與雜訊進入動力系統的特定結構；超出條件時是實用近似而非一般性定理。",
    },
    "related-work": {
        "task": "把 IT-MPC 放進取樣最佳化譜系，並選定交叉熵法作為基線。",
        "chain": "隨機搜尋共同點 → 權重分配差異 → CEM 菁英樣本更新 → 可檢驗比較。",
        "focus": "比較同時涉及理論更新規則與實作設定；不能把單一賽道結果直接外推成普遍優越性。",
    },
    "experimental-setup": {
        "task": "把理論變成可重現的車輛控制實驗。",
        "chain": "AutoRally 平台 → 兩種動力模型 → 賽道/速度成本 → 相同計算預算下比較。",
        "focus": "模型品質、成本函數與取樣數是三個主要混雜變因；閱讀結果時必須一起看。",
    },
    "results": {
        "task": "用大量圈次、成功率、圈速與特殊工況檢驗即時性、性能與韌性。",
        "chain": "整體績效 → 過彎行為 → 模型誤差 → 擾動拒斥 → 失敗模式。",
        "focus": "證據量很大，但平台是 1:5 車輛且場景封閉；對低速室內避障的可轉移性仍需另行驗證。",
    },
    "discussion": {
        "task": "回收論點、說明辨識力來源，並揭露取樣方法的限制。",
        "chain": "為何權重較有辨識力 → 為何 GPU 讓方法可行 → 失敗與未來改進。",
        "focus": "作者承認有限取樣、模型偏差與成本設計仍會導致失敗；這些正是工程部署的主要風險。",
    },
    "references": {
        "task": "提供理論、演算法與實驗脈絡的可追溯來源。",
        "chain": "最佳控制、路徑積分控制、取樣最佳化與自動駕駛實作。",
        "focus": "引文維持原文，不進行語義高亮。",
    },
}


HIGHLIGHT_RULES = [
    (
        "concept",
        [
            "information theoretic model predictive control",
            "stochastic optimal control",
            "model predictive control",
            "cross-entropy method",
            "importance sampling",
            "free-energy",
            "free energy",
            "KL-divergence",
            "inverse temperature",
            "path integral",
            "cost function",
        ],
    ),
    (
        "evidence",
        [
            r"\bover 100 kilometers\b",
            r"\bover 1700 total laps\b",
            r"\b40 Hz\b",
            r"\b\d+(?:\.\d+)?\s*(?:%|m/s|km|kilometers|laps|Hz)\b",
        ],
    ),
    (
        "counter",
        [
            "however",
            "despite",
            "although",
            "limitation",
            "failure mode",
            "difficult",
            "problematic",
            "not strictly necessary",
        ],
    ),
    (
        "method",
        [
            "derive",
            "algorithm",
            "sampling based",
            "sampling-based",
            "monte-carlo",
            "Monte Carlo",
            "dynamics model",
            "control update law",
            "GPU",
            "neural network model",
        ],
    ),
    (
        "thesis",
        [
            "we develop",
            "we demonstrate",
            "we show",
            "the contribution of this paper",
            "the advantage",
            "results show",
            "our approach",
        ],
    ),
]


def polish_translation(text: str) -> str:
    for source, target in TRANSLATION_FIXES.items():
        text = text.replace(source, target)
    text = re.sub(r"\s+([，。；：！？）])", r"\1", text)
    text = re.sub(r"（\s+", "（", text)
    text = re.sub(r"\s+", " ", text).strip()
    text = re.sub(r"([，。；：！？])\s+", r"\1", text)
    return text


def highlight_english(text: str) -> str:
    matches: list[tuple[int, int, str]] = []
    for category, patterns in HIGHLIGHT_RULES:
        for pattern in patterns:
            for match in re.finditer(pattern, text, re.I):
                if any(match.start() < end and match.end() > start for start, end, _ in matches):
                    continue
                matches.append((match.start(), match.end(), category))
    matches.sort()
    cursor = 0
    parts = []
    for start, end, category in matches:
        parts.append(html.escape(text[cursor:start]))
        parts.append(
            f'<mark class="hl-{category}">{html.escape(text[start:end])}</mark>'
        )
        cursor = end
    parts.append(html.escape(text[cursor:]))
    return "".join(parts)


def annotation_for(unit: Unit) -> tuple[str, str, str]:
    text = unit.english.lower()
    if unit.kind == "caption":
        return ("圖表定位", "視覺證據", "用圖表把抽象控制律連回平台、軌跡或量化結果。")
    if unit.kind == "subheading":
        return ("論證轉折", "結構標記", "此處切換子問題；先確認它如何服務本節的主要任務。")
    if unit.kind == "keywords":
        return ("概念索引", "術語邊界", "這些詞界定論文所屬的控制、學習與自駕研究脈絡。")
    if unit.kind == "reference":
        return ("來源追溯", "文獻基礎", "用於回查作者依賴的理論或比較方法。")
    if unit.section == "introduction":
        if "in this paper" in text or "contribution" in text:
            return ("核心主張", "缺口 → 解法", "作者把可處理性問題轉化為 GPU 可平行化的取樣控制問題。")
        if "traditionally" in text:
            return ("研究缺口", "限制既有方法", "此段排除離線策略與受限二次目標，為新框架建立必要性。")
        return ("問題建構", "背景 → 張力", "先承認分層方法成功，再指出高動態工況下的動力可行性缺口。")
    if unit.section == "preliminaries":
        return ("形式化契約", "定義 → 可推導性", "留意高斯雜訊、有限時域與成本拆分；後文結論都依賴這些假設。")
    if unit.section == "it-mpc":
        if "importance" in text or "weight" in text:
            return ("計算橋梁", "分佈 → 權重", "以重要性取樣把不可直接取得的最佳分佈改寫成有限樣本加權。")
        if "practical" in text or "smoothing" in text:
            return ("工程化處理", "理論 → 即時控制", "此處處理平滑、控制限制與有限樣本，不是純粹數學細節。")
        return ("核心推導", "界限 → 更新律", "檢查每一步使用的分佈、期望與近似，避免把等式與蒙地卡羅估計混為一談。")
    if unit.section == "stochastic-control":
        return ("理論對接", "新框架 ↔ 經典理論", "本節提供一致性論證，但等價成立需要控制仿射與雜訊結構條件。")
    if unit.section == "related-work":
        return ("比較基準", "同類方法對照", "CEM 與 IT-MPC 都取樣，但樣本淘汰和連續權重造成不同的資訊利用效率。")
    if unit.section == "experimental-setup":
        if any(term in text for term in ("cost", "parameter", "model")):
            return ("操作化", "理論量 → 實驗參數", "這些選擇直接影響結果；重現時應把模型、成本與取樣預算視為一組。")
        return ("實驗控制", "平台與程序", "用封閉賽道和固定速度目標建立可重複的比較環境。")
    if unit.section == "results":
        if any(term in text for term in ("failure", "problem", "unsafe", "difficult")):
            return ("反例與限制", "結果 → 邊界", "作者沒有只報成功案例；此段揭示有限取樣或模型偏差如何造成失效。")
        if re.search(r"\d", text):
            return ("實證證據", "主張 → 量化", "數據支持在此平台與設定下的比較結論，但不等於跨平台保證。")
        return ("結果解釋", "觀察 → 機制", "此段把軌跡行為連回權重更新、模型品質或成本設計。")
    if unit.section == "discussion":
        if any(term in text for term in ("however", "limitation", "failure", "future")):
            return ("讓步處理", "優勢 → 限制", "此處收斂外推範圍，指出取樣數、模型與成本仍是失敗來源。")
        return ("論點回收", "結果 → 結論", "作者將實驗差異解釋為資訊理論權重較能保留樣本間的成本結構。")
    return ("閱讀註記", "局部功能", "把此段放回本節的論證任務，確認它是定義、推導、證據或限制。")


def embedded_file(path: Path, mime: str) -> str:
    return f"data:{mime};base64," + base64.b64encode(path.read_bytes()).decode("ascii")


def render_section_header(section: str) -> str:
    title, subtitle = SECTION_TITLES[section]
    argument = SECTION_ARGUMENTS[section]
    return f"""
      <section class="section-band" id="{section}">
        <div>
          <p class="section-kicker">ARGUMENT MAP</p>
          <h2>{html.escape(title)}</h2>
          <p>{html.escape(subtitle)}</p>
        </div>
        <dl class="argument-grid">
          <div><dt>論證任務</dt><dd>{html.escape(argument["task"])}</dd></div>
          <div><dt>推理鏈</dt><dd>{html.escape(argument["chain"])}</dd></div>
          <div><dt>閱讀焦點</dt><dd>{html.escape(argument["focus"])}</dd></div>
        </dl>
      </section>
    """


def render_html(units: list[Unit], translations: list[str]) -> None:
    font_uri = embedded_file(TMP / "Lora-variable.ttf", "font/ttf")
    translation_map = {
        unit.source_index: polish_translation(translations[unit.translation_index])
        for unit in units
        if unit.translation_index is not None
    }

    body = []
    current_section = None
    equation_buffer: list[Unit] = []

    def flush_equations() -> None:
        if not equation_buffer:
            return
        page = equation_buffer[0].page
        equations = "".join(
            f"<div>{html.escape(item.english)}</div>" for item in equation_buffer
        )
        body.append(
            f"""
            <article class="reading-row formula-row" data-section="{equation_buffer[0].section}">
              <div class="passage">
                <div class="meta-line"><span>公式與表格原式</span><span>PDF p. {page}</span></div>
                <div class="equations">{equations}</div>
              </div>
              <aside class="annotation">
                <span class="role role-method">方法論</span>
                <h3>數學推導層</h3>
                <p>原式與符號保持不翻譯，避免改寫造成數學語義漂移；請配合前後定義閱讀。</p>
              </aside>
            </article>
            """
        )
        equation_buffer.clear()

    for unit in units:
        if unit.section != current_section:
            flush_equations()
            current_section = unit.section
            body.append(render_section_header(current_section))
        if unit.kind == "equation":
            equation_buffer.append(unit)
            continue
        flush_equations()
        role, relation, note = annotation_for(unit)
        translation = translation_map.get(unit.source_index, "")
        category = {
            "核心主張": "thesis",
            "實證證據": "evidence",
            "讓步處理": "counter",
            "反例與限制": "counter",
            "核心推導": "method",
            "計算橋梁": "method",
            "工程化處理": "method",
            "形式化契約": "concept",
            "概念索引": "concept",
        }.get(role, "neutral")
        body.append(
            f"""
            <article class="reading-row" data-section="{unit.section}">
              <div class="passage">
                <div class="meta-line"><span>{html.escape(unit.kind.upper())}</span><span>PDF p. {unit.page}</span></div>
                <p class="english">{highlight_english(unit.english)}</p>
                {f'<div class="translation"><p>{html.escape(translation)}</p></div>' if translation else ''}
              </div>
              <aside class="annotation">
                <span class="role role-{category}">{html.escape(role)}</span>
                <h3>{html.escape(relation)}</h3>
                <p>{html.escape(note)}</p>
              </aside>
            </article>
            """
        )
    flush_equations()

    page_images = []
    for page in range(1, 21):
        path = RENDER_DIR / f"page-{page:02d}.jpg"
        page_images.append(
            f'<figure><img loading="lazy" src="{embedded_file(path, "image/jpeg")}" '
            f'alt="原始論文第 {page} 頁"><figcaption>原始 PDF 第 {page} 頁</figcaption></figure>'
        )

    nav_items = "".join(
        f'<a href="#{key}">{html.escape(value[0])}</a>'
        for key, value in SECTION_TITLES.items()
    )

    document = f"""<!doctype html>
<html lang="zh-Hant">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Information Theoretic Model Predictive Control｜雙語論證批註</title>
  <style>
    @font-face {{
      font-family: "Lora Embedded";
      src: url("{font_uri}") format("truetype");
      font-style: normal;
      font-weight: 400 700;
      font-display: swap;
    }}
    :root {{
      --ink: #172033;
      --muted: #617087;
      --line: #d9dee7;
      --paper: #ffffff;
      --canvas: #f3f5f7;
      --cream: #fbf7eb;
      --nav: #111827;
      --yellow: #fff0a6;
      --red: #ffd5d2;
      --blue: #cfe8ff;
      --green: #d7f1dc;
      --purple: #eadcff;
      --content: 1560px;
    }}
    * {{ box-sizing: border-box; }}
    html {{ scroll-behavior: smooth; scroll-padding-top: 156px; }}
    body {{
      margin: 0;
      color: var(--ink);
      background: var(--canvas);
      font-family: -apple-system, BlinkMacSystemFont, "PingFang TC", "Noto Sans TC", sans-serif;
      line-height: 1.72;
      letter-spacing: 0;
    }}
    .topbar {{
      position: sticky;
      top: 0;
      z-index: 20;
      color: white;
      background: var(--nav);
      border-bottom: 1px solid #2c3443;
    }}
    .topbar-inner {{
      width: min(var(--content), calc(100% - 40px));
      min-height: 92px;
      margin: 0 auto;
      display: grid;
      grid-template-columns: minmax(320px, 1fr) auto;
      gap: 32px;
      align-items: center;
    }}
    .paper-title h1 {{
      margin: 0;
      font-family: "Lora Embedded", Georgia, serif;
      font-size: 22px;
      line-height: 1.25;
      font-weight: 650;
    }}
    .paper-title p {{ margin: 4px 0 0; color: #adb8ca; font-size: 13px; }}
    .legend {{ display: flex; flex-wrap: wrap; justify-content: flex-end; gap: 8px 14px; max-width: 650px; }}
    .legend span {{ display: inline-flex; align-items: center; gap: 7px; font-size: 12px; white-space: nowrap; }}
    .swatch {{ width: 15px; height: 15px; border: 1px solid rgba(255,255,255,.28); }}
    .section-nav {{
      position: sticky;
      top: 92px;
      z-index: 19;
      overflow-x: auto;
      white-space: nowrap;
      background: #fff;
      border-bottom: 1px solid var(--line);
      scrollbar-width: thin;
    }}
    .section-nav div {{
      width: min(var(--content), calc(100% - 40px));
      margin: 0 auto;
      display: flex;
      min-height: 54px;
      align-items: center;
      gap: 4px;
    }}
    .section-nav a {{
      color: #334155;
      text-decoration: none;
      font-size: 13px;
      font-weight: 700;
      padding: 8px 12px;
      border-bottom: 2px solid transparent;
    }}
    .section-nav a:hover, .section-nav a.active {{ color: #0f172a; border-color: #2563eb; }}
    main {{
      width: min(var(--content), calc(100% - 40px));
      margin: 30px auto 80px;
      background: var(--paper);
      border: 1px solid var(--line);
      box-shadow: 0 12px 32px rgba(15, 23, 42, .07);
    }}
    .hero {{
      padding: 44px 48px 40px;
      border-bottom: 1px solid var(--line);
      display: grid;
      grid-template-columns: minmax(0, 1fr) 420px;
      gap: 44px;
      align-items: end;
    }}
    .hero .eyebrow, .section-kicker {{
      margin: 0 0 8px;
      color: #2563eb;
      font: 800 11px/1.2 -apple-system, BlinkMacSystemFont, sans-serif;
      letter-spacing: .12em;
    }}
    .hero h2 {{
      margin: 0;
      max-width: 920px;
      font-family: "Lora Embedded", Georgia, serif;
      font-size: 38px;
      line-height: 1.18;
    }}
    .hero .authors {{ margin: 16px 0 0; color: var(--muted); }}
    .hero-summary {{ border-left: 3px solid #2563eb; padding-left: 22px; }}
    .hero-summary strong {{ display: block; margin-bottom: 6px; }}
    .hero-summary p {{ margin: 0; color: #46556d; font-size: 14px; }}
    .section-band {{
      scroll-margin-top: 160px;
      padding: 34px 48px;
      background: #eef2f6;
      border-top: 1px solid var(--line);
      border-bottom: 1px solid var(--line);
      display: grid;
      grid-template-columns: 330px minmax(0, 1fr);
      gap: 48px;
    }}
    .section-band:first-of-type {{ border-top: 0; }}
    .section-band h2 {{ margin: 0; font-size: 26px; line-height: 1.25; }}
    .section-band p {{ margin: 8px 0 0; color: var(--muted); font-size: 14px; }}
    .argument-grid {{ margin: 0; display: grid; grid-template-columns: repeat(3, 1fr); gap: 24px; }}
    .argument-grid div {{ border-left: 1px solid #c8d0dc; padding-left: 16px; }}
    .argument-grid dt {{ font-size: 12px; color: #5d6d83; font-weight: 800; }}
    .argument-grid dd {{ margin: 5px 0 0; font-size: 14px; line-height: 1.55; }}
    .reading-row {{
      display: grid;
      grid-template-columns: minmax(0, 2.15fr) minmax(280px, .85fr);
      border-bottom: 1px solid var(--line);
    }}
    .passage {{ padding: 34px 48px 38px; min-width: 0; }}
    .annotation {{
      padding: 34px 32px;
      background: #f8fafc;
      border-left: 1px solid var(--line);
    }}
    .meta-line {{
      display: flex;
      justify-content: space-between;
      margin-bottom: 12px;
      color: #8090a4;
      font-size: 11px;
      font-weight: 800;
    }}
    .english {{
      margin: 0;
      font-family: "Lora Embedded", Georgia, serif;
      font-size: 17px;
      line-height: 1.86;
      color: #1d2738;
    }}
    mark {{ color: inherit; padding: .05em .12em; box-decoration-break: clone; -webkit-box-decoration-break: clone; }}
    .hl-thesis {{ background: var(--yellow); }}
    .hl-concept {{ background: var(--red); }}
    .hl-evidence {{ background: var(--blue); }}
    .hl-counter {{ background: var(--green); }}
    .hl-method {{ background: var(--purple); }}
    .translation {{
      margin-top: 22px;
      padding: 17px 20px;
      background: var(--cream);
      border-top: 1px solid #e4decf;
      border-left: 3px solid #b9a778;
    }}
    .translation p {{ margin: 0; font-size: 15px; line-height: 1.82; color: #3d4653; }}
    .role {{
      display: inline-block;
      padding: 3px 7px;
      border-radius: 4px;
      color: #334155;
      background: #e5e7eb;
      font-size: 11px;
      font-weight: 800;
    }}
    .role-thesis {{ background: var(--yellow); }}
    .role-concept {{ background: var(--red); }}
    .role-evidence {{ background: var(--blue); }}
    .role-counter {{ background: var(--green); }}
    .role-method {{ background: var(--purple); }}
    .annotation h3 {{ margin: 13px 0 7px; font-size: 16px; line-height: 1.3; }}
    .annotation p {{ margin: 0; color: #5b687a; font-size: 13px; line-height: 1.65; }}
    .equations {{
      overflow-x: auto;
      padding: 20px;
      color: #111827;
      background: #f6f7f9;
      border: 1px solid #e2e7ee;
      font: 15px/1.8 "SFMono-Regular", Consolas, monospace;
    }}
    .equations div + div {{ margin-top: 5px; }}
    .source-appendix {{
      padding: 38px 48px 52px;
      background: #eef2f6;
      border-top: 1px solid var(--line);
    }}
    .source-appendix summary {{ cursor: pointer; font-size: 19px; font-weight: 800; }}
    .source-appendix > p {{ color: var(--muted); font-size: 14px; }}
    .page-grid {{ margin-top: 26px; display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 24px; }}
    .page-grid figure {{ margin: 0; background: #fff; border: 1px solid var(--line); }}
    .page-grid img {{ display: block; width: 100%; height: auto; }}
    .page-grid figcaption {{ padding: 8px 12px; color: var(--muted); font-size: 12px; border-top: 1px solid var(--line); }}
    footer {{ padding: 24px 48px; color: #66758a; background: #fff; font-size: 12px; }}
    @media (max-width: 980px) {{
      .topbar-inner {{ grid-template-columns: 1fr; gap: 12px; padding: 16px 0; }}
      .legend {{ justify-content: flex-start; }}
      .section-nav {{ top: 140px; }}
      html {{ scroll-padding-top: 204px; }}
      .hero, .section-band, .reading-row {{ grid-template-columns: 1fr; }}
      .hero {{ gap: 24px; }}
      .section-band {{ gap: 22px; }}
      .argument-grid {{ grid-template-columns: 1fr; gap: 12px; }}
      .annotation {{ border-left: 0; border-top: 1px solid var(--line); }}
    }}
    @media (max-width: 640px) {{
      .topbar {{ position: static; }}
      .topbar-inner, .section-nav div, main {{ width: 100%; }}
      main {{ margin-top: 0; border-left: 0; border-right: 0; }}
      .topbar-inner {{ padding: 14px 18px; }}
      .legend {{ display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); }}
      .section-nav {{ top: 0; }}
      .section-nav div {{ padding: 0 8px; }}
      .hero, .section-band, .passage, .annotation, .source-appendix, footer {{ padding-left: 22px; padding-right: 22px; }}
      .hero h2 {{ font-size: 29px; }}
      .english {{ font-size: 16px; }}
      .page-grid {{ grid-template-columns: 1fr; }}
    }}
    @media print {{
      .topbar, .section-nav {{ position: static; }}
      main {{ width: 100%; margin: 0; border: 0; box-shadow: none; }}
      .reading-row {{ break-inside: avoid; }}
      .source-appendix {{ display: none; }}
    }}
  </style>
</head>
<body>
  <header class="topbar">
    <div class="topbar-inner">
      <div class="paper-title">
        <h1>Information Theoretic Model Predictive Control</h1>
        <p>自主移動機器人導航研究｜MPPI 基礎閱讀單元｜繁體中文批註版</p>
      </div>
      <div class="legend" aria-label="分析維度圖例">
        <span><i class="swatch" style="background:var(--yellow)"></i>核心論點 / thesis</span>
        <span><i class="swatch" style="background:var(--red)"></i>關鍵概念 / 術語</span>
        <span><i class="swatch" style="background:var(--blue)"></i>實證證據 / 數據</span>
        <span><i class="swatch" style="background:var(--green)"></i>讓步 / 反駁處理</span>
        <span><i class="swatch" style="background:var(--purple)"></i>方法論說明</span>
      </div>
    </div>
  </header>
  <nav class="section-nav" aria-label="章節導航"><div>{nav_items}</div></nav>
  <main>
    <section class="hero">
      <div>
        <p class="eyebrow">COMPLETE BILINGUAL ARGUMENT ANNOTATION</p>
        <h2>Information Theoretic Model Predictive Control: Theory and Applications to Autonomous Driving</h2>
        <p class="authors">Grady Williams · Paul Drews · Brian Goldfain · James M. Rehg · Evangelos A. Theodorou</p>
      </div>
      <div class="hero-summary">
        <strong>全文總論證</strong>
        <p>作者先把高動態自駕描述為非線性、非凸且必須即時求解的最佳控制問題，再以自由能與 KL 散度推導可平行化的取樣更新律，最後用 AutoRally 大量實驗和 CEM 基線檢驗性能、韌性與失敗邊界。</p>
      </div>
    </section>
    {"".join(body)}
    <details class="source-appendix">
      <summary>核對原始 PDF 全 20 頁</summary>
      <p>此區內嵌原始頁面影像，用於核對公式、表格、圖說與雙欄閱讀順序；不依賴外部檔案。</p>
      <div class="page-grid">{"".join(page_images)}</div>
    </details>
    <footer>
      來源：Williams et al., arXiv:1707.02342v1 (2017)。中文為繁體學術意譯與術語校訂；公式與參考文獻保留原文。
    </footer>
  </main>
  <script>
    const links = [...document.querySelectorAll('.section-nav a')];
    const sections = links.map(link => document.querySelector(link.getAttribute('href'))).filter(Boolean);
    function updateActiveSection() {{
      const threshold = innerWidth <= 640 ? 82 : 170;
      let current = sections[0];
      for (const section of sections) {{
        if (section.getBoundingClientRect().top <= threshold) current = section;
      }}
      links.forEach(link => link.classList.toggle('active', link.getAttribute('href') === '#' + current.id));
    }}
    addEventListener('scroll', updateActiveSection, {{ passive: true }});
    addEventListener('resize', updateActiveSection);
    updateActiveSection();
  </script>
</body>
</html>
"""
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_PATH.write_text(document, encoding="utf-8")
    print(f"Wrote {OUTPUT_PATH} ({OUTPUT_PATH.stat().st_size / 1024 / 1024:.1f} MiB).")


def main() -> None:
    units = extract_units()
    write_translation_input(units)
    print(f"Extracted {len(units)} reading units.")
    print(
        "Translation requests:",
        sum(unit.translation_index is not None for unit in units),
    )
    if TRANSLATIONS_PATH.exists():
        translations = json.loads(TRANSLATIONS_PATH.read_text(encoding="utf-8"))
        expected = sum(unit.translation_index is not None for unit in units)
        if len(translations) < expected:
            raise ValueError(f"Only {len(translations)} translations for {expected} passages")
        render_html(units, translations)


if __name__ == "__main__":
    main()
