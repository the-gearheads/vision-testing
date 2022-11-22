#include "ApriltagDetect.h"

ApriltagDetect::ApriltagDetect(json config)
{
  this->tag_family = tag16h5_create();
  this->detector = apriltag_detector_create();
  apriltag_detector_add_family(detector, tag_family);

  detector->quad_decimate = config.value("apriltag_quad_decimate", 2);
  detector->quad_sigma = config.value("apriltag_blur", 0);
  detector->nthreads = config.value("apriltag_threads", 0);
  detector->debug = config.value("apriltag_debug", false);
  detector->refine_edges = config.value("apriltag_refine_edges", true);
  if (!detector->nthreads)
    detector->nthreads = std::thread::hardware_concurrency();
  if (!detector->nthreads)
    detector->nthreads = 1;
  printf("Using %d threads\n", detector->nthreads);
}

ApriltagDetect::~ApriltagDetect()
{
  apriltag_detector_destroy(detector);
  tag16h5_destroy(tag_family);
}

void ApriltagDetect::execute(Mat img)
{
  Mat greyImg;
  cvtColor(img, greyImg, COLOR_BGR2GRAY);

  // Make an image_u8_t header for the Mat data
  image_u8_t frame = { .width = greyImg.cols, .height = greyImg.rows, .stride = greyImg.cols, .buf = greyImg.data };

  zarray_t* detections = apriltag_detector_detect(detector, &frame);

  /* Do stuff with detections here */
  // Draw detection outlines
  for (int i = 0; i < zarray_size(detections); i++)
  {
    apriltag_detection_t* det;
    zarray_get(detections, i, &det);
    line(img, Point(det->p[0][0], det->p[0][1]), Point(det->p[1][0], det->p[1][1]), Scalar(0, 0xff, 0), 2);
    line(img, Point(det->p[0][0], det->p[0][1]), Point(det->p[3][0], det->p[3][1]), Scalar(0, 0, 0xff), 2);
    line(img, Point(det->p[1][0], det->p[1][1]), Point(det->p[2][0], det->p[2][1]), Scalar(0xff, 0, 0), 2);
    line(img, Point(det->p[2][0], det->p[2][1]), Point(det->p[3][0], det->p[3][1]), Scalar(0xff, 0, 0), 2);

    std::stringstream ss;
    ss << det->id;
    String text = ss.str();
    double fontscale = 1.0;
    int baseline;
    Size textsize = getTextSize(text, FONT, fontscale, 2, &baseline);
    putText(img, text, Point(det->c[0] - textsize.width / 2, det->c[1] + textsize.height / 2), FONT, fontscale,
            Scalar(0xff, 0x99, 0), 2);
  }
  apriltag_detections_destroy(detections);
}