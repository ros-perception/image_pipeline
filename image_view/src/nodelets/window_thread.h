#ifndef IMAGE_VIEW_WINDOW_THREAD_H
#define IMAGE_VIEW_WINDOW_THREAD_H

namespace image_view {

// Makes absolutely sure we only start the OpenCV window thread once
void startWindowThread();

} // namespace image_view

#endif
