#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <errno.h>

#define DEVICE "/dev/video0"
#define WIDTH 640
#define HEIGHT 480
#define BUFFER_COUNT 4

struct buffer {
    void   *start;
    size_t length;
};

int main() {
    int fd;
    struct v4l2_format fmt;
    struct v4l2_requestbuffers req;
    struct v4l2_buffer buf;
    struct buffer buffers[BUFFER_COUNT];

    // 1. Open device
    fd = open(DEVICE, O_RDWR);
    if (fd < 0) {
        perror("Cannot open device");
        return 1;
    }

    // 2. Set format
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = WIDTH;
    fmt.fmt.pix.height = HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        perror("Failed to set format");
        return 1;
    }

    // 3. Request buffer allocation
    memset(&req, 0, sizeof(req));
    req.count = BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("Failed to request buffers");
        return 1;
    }

    // 4. Map the buffers
    for (int i = 0; i < BUFFER_COUNT; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
            perror("Failed to query buffer");
            return 1;
        }

        buffers[i].length = buf.length;
        buffers[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                                fd, buf.m.offset);

        if (buffers[i].start == MAP_FAILED) {
            perror("mmap");
            return 1;
        }

        // Queue buffer for capture
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            perror("Failed to queue buffer");
            return 1;
        }
    }

    // 5. Start streaming (only ONCE!)
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        perror("Failed to start streaming");
        return 1;
    }

    // 6. Capture loop
    for (int i = 0; i < 10; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        // Dequeue (wait for frame)
        if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
            perror("Failed to dequeue buffer");
            return 1;
        }

        printf("Captured frame %d, length = %d bytes\n", i, buf.bytesused);

        // Requeue buffer
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            perror("Failed to requeue buffer");
            return 1;
        }
    }

    // 7. Stop streaming (only ONCE!)
    if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
        perror("Failed to stop streaming");
        return 1;
    }

    // 8. Cleanup
    for (int i = 0; i < BUFFER_COUNT; i++) {
        munmap(buffers[i].start, buffers[i].length);
    }
    close(fd);

    return 0;
}