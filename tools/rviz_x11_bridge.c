#define _GNU_SOURCE

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/extensions/shape.h>
#include <X11/extensions/XTest.h>

#include <errno.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static volatile sig_atomic_t running = 1;
static int debug_frames = 0;
static int debug_sources = 0;
static unsigned long debug_nonblack = 0;
static int debug_input_events = 0;
static int debug_grab_events = 0;

static int bridge_x_error(Display *display, XErrorEvent *event) {
  if (debug_frames < 3) {
    char error_text[256];
    XGetErrorText(display, event->error_code, error_text, sizeof(error_text));
    fprintf(stderr, "X11 error request=%u code=%u (%s) resource=0x%lx\n",
            event->request_code, event->error_code, error_text,
            event->resourceid);
    fflush(stderr);
  }
  return 0;
}

static void stop_bridge(int signal_number) {
  (void)signal_number;
  running = 0;
}

static unsigned long mask_shift(unsigned long mask) {
  unsigned long shift = 0;
  while (mask != 0 && (mask & 1UL) == 0) {
    mask >>= 1;
    ++shift;
  }
  return shift;
}

static unsigned long mask_bits(unsigned long mask) {
  unsigned long bits = 0;
  while (mask != 0) {
    bits += mask & 1UL;
    mask >>= 1;
  }
  return bits;
}

static unsigned long convert_component(unsigned long pixel, unsigned long mask) {
  if (mask == 0) {
    return 0;
  }
  unsigned long shift = mask_shift(mask);
  unsigned long bits = mask_bits(mask);
  unsigned long value = (pixel & mask) >> shift;
  unsigned long max_value = (1UL << bits) - 1UL;
  return (value * 255UL + max_value / 2UL) / max_value;
}

static unsigned long make_pixel(unsigned long red, unsigned long green,
                                unsigned long blue, Visual *visual) {
  unsigned long pixel = 0;
  if (visual->red_mask != 0) {
    unsigned long max_value = visual->red_mask >> mask_shift(visual->red_mask);
    pixel |= ((red * max_value + 127UL) / 255UL) << mask_shift(visual->red_mask);
  }
  if (visual->green_mask != 0) {
    unsigned long max_value = visual->green_mask >> mask_shift(visual->green_mask);
    pixel |= ((green * max_value + 127UL) / 255UL) << mask_shift(visual->green_mask);
  }
  if (visual->blue_mask != 0) {
    unsigned long max_value = visual->blue_mask >> mask_shift(visual->blue_mask);
    pixel |= ((blue * max_value + 127UL) / 255UL) << mask_shift(visual->blue_mask);
  }
  return pixel;
}

typedef struct {
  Display *display;
  Window bridge;
  Window target;
  int screen;
  GC gc;
  Visual *root_visual;
  int root_depth;
  int width;
  int height;
  Bool pointer_grabbed;
  Bool keyboard_grabbed;
} BridgeContext;

static void grab_bridge_inputs(BridgeContext *ctx) {
  if (!ctx->pointer_grabbed) {
    int result = XGrabPointer(
        ctx->display, ctx->bridge, False,
        PointerMotionMask | ButtonPressMask | ButtonReleaseMask,
        GrabModeAsync, GrabModeAsync, None, None, CurrentTime);
    ctx->pointer_grabbed = (result == GrabSuccess);
    if (debug_grab_events < 3) {
      fprintf(stderr, "pointer_grab=%d result=%d\n", ctx->pointer_grabbed, result);
      fflush(stderr);
      ++debug_grab_events;
    }
  }
  if (!ctx->keyboard_grabbed) {
    int result = XGrabKeyboard(ctx->display, ctx->bridge, False,
                               GrabModeAsync, GrabModeAsync, CurrentTime);
    ctx->keyboard_grabbed = (result == GrabSuccess);
    if (debug_grab_events < 3) {
      fprintf(stderr, "keyboard_grab=%d result=%d\n", ctx->keyboard_grabbed, result);
      fflush(stderr);
      ++debug_grab_events;
    }
  }
  XSync(ctx->display, False);
}

static void forward_pointer_event(BridgeContext *ctx, int x_root, int y_root,
                                  unsigned int button, Bool pressed) {
  // The VNC input event lands on the bridge.  Temporarily lower it so XTest
  // targets the real RViz window, then restore the bridge for the next frame.
  if (ctx->pointer_grabbed) {
    XUngrabPointer(ctx->display, CurrentTime);
    ctx->pointer_grabbed = False;
  }
  XLowerWindow(ctx->display, ctx->bridge);
  XSync(ctx->display, False);
  XTestFakeMotionEvent(ctx->display, ctx->screen, x_root, y_root, CurrentTime);
  if (button != 0) {
    XTestFakeButtonEvent(ctx->display, button, pressed, CurrentTime);
  }
  XFlush(ctx->display);
  XSync(ctx->display, False);
  XRaiseWindow(ctx->display, ctx->bridge);
  grab_bridge_inputs(ctx);
  XSync(ctx->display, False);
}

static void forward_key_event(BridgeContext *ctx, unsigned int keycode,
                              Bool pressed) {
  if (ctx->keyboard_grabbed) {
    XUngrabKeyboard(ctx->display, CurrentTime);
    ctx->keyboard_grabbed = False;
  }
  XSetInputFocus(ctx->display, ctx->target, RevertToPointerRoot, CurrentTime);
  XTestFakeKeyEvent(ctx->display, keycode, pressed, CurrentTime);
  XFlush(ctx->display);
  XSync(ctx->display, False);
  grab_bridge_inputs(ctx);
}

static void process_bridge_input(BridgeContext *ctx) {
  XEvent event;
  while (XPending(ctx->display) > 0) {
    XNextEvent(ctx->display, &event);
    if (event.xany.window != ctx->bridge) {
      continue;
    }
    if (event.type == MotionNotify) {
      forward_pointer_event(ctx, event.xmotion.x_root, event.xmotion.y_root,
                             0, False);
    } else if (event.type == ButtonPress || event.type == ButtonRelease) {
      if (debug_input_events < 30) {
        fprintf(stderr, "input button=%u pressed=%d root=%d,%d\n",
                event.xbutton.button, event.type == ButtonPress,
                event.xbutton.x_root, event.xbutton.y_root);
        fflush(stderr);
        ++debug_input_events;
      }
      forward_pointer_event(ctx, event.xbutton.x_root, event.xbutton.y_root,
                             event.xbutton.button, event.type == ButtonPress);
    } else if (event.type == KeyPress || event.type == KeyRelease) {
      if (debug_input_events < 60) {
        KeySym keysym = XLookupKeysym(&event.xkey, 0);
        fprintf(stderr, "input keycode=%u keysym=0x%lx pressed=%d\n",
                event.xkey.keycode, (unsigned long)keysym,
                event.type == KeyPress);
        fflush(stderr);
        ++debug_input_events;
      }
      forward_key_event(ctx, event.xkey.keycode, event.type == KeyPress);
    }
  }
}

static void copy_image_to_frame(BridgeContext *ctx, XImage *source,
                                Visual *source_visual, XImage *frame,
                                int destination_x, int destination_y) {
  int start_x = destination_x < 0 ? -destination_x : 0;
  int start_y = destination_y < 0 ? -destination_y : 0;
  int end_x = source->width;
  int end_y = source->height;
  if (destination_x + end_x > ctx->width) {
    end_x = ctx->width - destination_x;
  }
  if (destination_y + end_y > ctx->height) {
    end_y = ctx->height - destination_y;
  }
  if (start_x >= end_x || start_y >= end_y) {
    return;
  }

  for (int y = start_y; y < end_y; ++y) {
    for (int x = start_x; x < end_x; ++x) {
      unsigned long source_pixel = XGetPixel(source, x, y);
      unsigned long red = convert_component(source_pixel, source_visual->red_mask);
      unsigned long green = convert_component(source_pixel, source_visual->green_mask);
      unsigned long blue = convert_component(source_pixel, source_visual->blue_mask);
      XPutPixel(frame, destination_x + x, destination_y + y,
                make_pixel(red, green, blue, ctx->root_visual));
    }
  }
}

static void capture_window(BridgeContext *ctx, Window window, XImage *frame,
                           int offset_x, int offset_y, int level) {
  if (level > 12) {
    return;
  }

  XWindowAttributes attributes;
  int got_attributes = XGetWindowAttributes(ctx->display, window, &attributes);
  if (!got_attributes || attributes.map_state != IsViewable ||
      attributes.width <= 0 || attributes.height <= 0 || attributes.visual == NULL) {
    if (debug_frames < 3 && level == 0) {
      fprintf(stderr, "capture skipped got=%d map=%d size=%dx%d visual=%p\n",
              got_attributes, got_attributes ? attributes.map_state : -1,
              got_attributes ? attributes.width : -1,
              got_attributes ? attributes.height : -1,
              got_attributes ? (void *)attributes.visual : NULL);
      fflush(stderr);
    }
    return;
  }

  if (debug_frames < 3 && level == 0) {
    fprintf(stderr, "target map=%d size=%dx%d depth=%d visual=%p\n",
            attributes.map_state, attributes.width, attributes.height,
            attributes.depth, (void *)attributes.visual);
    fflush(stderr);
  }

  XImage *source = XGetImage(ctx->display, window, 0, 0,
                             (unsigned int)attributes.width,
                             (unsigned int)attributes.height, AllPlanes,
                             ZPixmap);
  if (source != NULL) {
    if (debug_frames < 3) {
      ++debug_sources;
    }
    copy_image_to_frame(ctx, source, attributes.visual, frame, offset_x, offset_y);
    XDestroyImage(source);
  } else if (debug_frames < 3) {
    fprintf(stderr, "XGetImage failed window=0x%lx level=%d\n",
            (unsigned long)window, level);
    fflush(stderr);
  }

  Window root_return = 0;
  Window parent_return = 0;
  Window *children = NULL;
  unsigned int child_count = 0;
  if (!XQueryTree(ctx->display, window, &root_return, &parent_return,
                  &children, &child_count)) {
    return;
  }

  for (unsigned int i = 0; i < child_count; ++i) {
    Window child_root = 0;
    int child_x = 0;
    int child_y = 0;
    unsigned int child_width = 0;
    unsigned int child_height = 0;
    unsigned int child_border = 0;
    unsigned int child_depth = 0;
    if (XGetGeometry(ctx->display, children[i], &child_root, &child_x, &child_y,
                     &child_width, &child_height, &child_border, &child_depth)) {
      capture_window(ctx, children[i], frame, offset_x + child_x,
                     offset_y + child_y, level + 1);
    }
  }
  if (children != NULL) {
    XFree(children);
  }
}

static int window_geometry(Display *display, Window target, Window root,
                           int *x, int *y, int *width, int *height) {
  XWindowAttributes attributes;
  if (!XGetWindowAttributes(display, target, &attributes) ||
      attributes.width <= 0 || attributes.height <= 0) {
    return 0;
  }
  Window translated_child = 0;
  if (!XTranslateCoordinates(display, target, root, 0, 0, x, y,
                             &translated_child)) {
    return 0;
  }
  *width = attributes.width;
  *height = attributes.height;
  return 1;
}

static Window parse_window_id(const char *text) {
  errno = 0;
  unsigned long value = strtoul(text, NULL, 0);
  if (errno != 0 || value == 0) {
    fprintf(stderr, "invalid X11 window id: %s\n", text);
    exit(2);
  }
  return (Window)value;
}

static void request_window_activation(Display *display, Window root,
                                      Window target) {
  Atom active_atom = XInternAtom(display, "_NET_ACTIVE_WINDOW", False);
  if (active_atom == None) {
    return;
  }
  XClientMessageEvent event;
  memset(&event, 0, sizeof(event));
  event.type = ClientMessage;
  event.window = target;
  event.message_type = active_atom;
  event.format = 32;
  event.data.l[0] = 2;  // pager/source indication: application request
  event.data.l[1] = CurrentTime;
  event.data.l[2] = (long)target;
  XSendEvent(display, root, False,
             SubstructureRedirectMask | SubstructureNotifyMask,
             (XEvent *)&event);
}

int main(int argc, char **argv) {
  if (argc < 2 || argc > 3) {
    fprintf(stderr, "usage: %s TARGET_WINDOW [ID_FILE]\n", argv[0]);
    return 2;
  }

  signal(SIGINT, stop_bridge);
  signal(SIGTERM, stop_bridge);

  Display *display = XOpenDisplay(NULL);
  if (display == NULL) {
    fprintf(stderr, "cannot open DISPLAY\n");
    return 1;
  }
  XSetErrorHandler(bridge_x_error);

  Window target = parse_window_id(argv[1]);
  int screen = DefaultScreen(display);
  Window root = RootWindow(display, screen);
  Visual *root_visual = DefaultVisual(display, screen);
  int root_depth = DefaultDepth(display, screen);
  int x = 0;
  int y = 0;
  int width = 0;
  int height = 0;
  if (!window_geometry(display, target, root, &x, &y, &width, &height)) {
    fprintf(stderr, "target window is unavailable\n");
    XCloseDisplay(display);
    return 1;
  }

  XSetWindowAttributes attributes;
  memset(&attributes, 0, sizeof(attributes));
  attributes.override_redirect = True;
  attributes.background_pixel = BlackPixel(display, screen);
  Window bridge = XCreateWindow(
      display, root, x, y, (unsigned int)width, (unsigned int)height, 0,
      (unsigned int)root_depth, InputOutput, root_visual,
      CWOverrideRedirect | CWBackPixel, &attributes);
  if (bridge == 0) {
    fprintf(stderr, "cannot create bridge window\n");
    XCloseDisplay(display);
    return 1;
  }

  XStoreName(display, bridge, "RViz display bridge");
  attributes.event_mask = PointerMotionMask | ButtonPressMask |
                          ButtonReleaseMask | KeyPressMask | KeyReleaseMask;
  XSelectInput(display, bridge, attributes.event_mask);
  // RViz can remain in the X11 tree but become unmapped after a VNC client
  // reconnects.  Explicitly map it here so the OpenGL children become
  // viewable again before the first capture.
  XMapRaised(display, target);
  request_window_activation(display, root, target);
  XMapRaised(display, target);
  XSync(display, False);
  XSetInputFocus(display, target, RevertToPointerRoot, CurrentTime);
  XSync(display, False);

  // Keep the bridge below RViz.  It is a VNC framebuffer sink, not an
  // on-screen overlay; leaving it below the OpenGL window preserves the
  // renderer's visible front buffer while XGetImage reads the real RViz
  // child windows.
  XMapRaised(display, bridge);
  XFlush(display);

  if (argc == 3) {
    FILE *id_file = fopen(argv[2], "w");
    if (id_file != NULL) {
      fprintf(id_file, "0x%lx\n", (unsigned long)bridge);
      fclose(id_file);
    }
  }
  printf("bridge_window=0x%lx\n", (unsigned long)bridge);
  fflush(stdout);

  BridgeContext context = {
      .display = display,
      .bridge = bridge,
      .target = target,
      .screen = screen,
      .gc = XCreateGC(display, bridge, 0, NULL),
      .root_visual = root_visual,
      .root_depth = root_depth,
      .width = width,
      .height = height,
      .pointer_grabbed = False,
      .keyboard_grabbed = False,
  };

  grab_bridge_inputs(&context);

  while (running) {
    int new_x = 0;
    int new_y = 0;
    int new_width = 0;
    int new_height = 0;
    if (!window_geometry(display, target, root, &new_x, &new_y, &new_width,
                         &new_height)) {
      break;
    }
    if (new_x != x || new_y != y || new_width != width || new_height != height) {
      x = new_x;
      y = new_y;
      width = new_width;
      height = new_height;
      context.width = width;
      context.height = height;
      XMoveResizeWindow(display, bridge, x, y, (unsigned int)width,
                        (unsigned int)height);
      XMapRaised(display, bridge);
      XFlush(display);
    }

    process_bridge_input(&context);

    XImage *frame = XCreateImage(display, root_visual, (unsigned int)root_depth,
                                 ZPixmap, 0, NULL, (unsigned int)width,
                                 (unsigned int)height, 32, 0);
    if (frame == NULL) {
      break;
    }
    frame->data = calloc(1, (size_t)frame->bytes_per_line * (size_t)height);
    if (frame->data == NULL) {
      XDestroyImage(frame);
      break;
    }

    XLowerWindow(display, bridge);
    XSync(display, False);
    capture_window(&context, target, frame, 0, 0, 0);
    if (debug_frames < 3) {
      // Sample the composed frame so the log distinguishes a capture failure
      // from a VNC transport failure without flooding the container log.
      for (int sample_y = 0; sample_y < height; sample_y += 16) {
        for (int sample_x = 0; sample_x < width; sample_x += 16) {
          if (XGetPixel(frame, sample_x, sample_y) != 0) {
            ++debug_nonblack;
          }
        }
      }
      fprintf(stderr, "frame=%d captured_sources=%d sampled_nonblack=%lu size=%dx%d\n",
              debug_frames, debug_sources, debug_nonblack, width, height);
      fflush(stderr);
      ++debug_frames;
    }
    XPutImage(display, bridge, context.gc, frame, 0, 0, 0, 0,
              (unsigned int)width, (unsigned int)height);
    XDestroyImage(frame);
    XRaiseWindow(display, bridge);
    XFlush(display);
    usleep(33333);
  }

  if (argc == 3) {
    unlink(argv[2]);
  }
  XDestroyWindow(display, bridge);
  XFreeGC(display, context.gc);
  XCloseDisplay(display);
  return 0;
}
