#include "ApriltagDetect.h"
#include "Config.h"

ApriltagDetect::ApriltagDetect(json config, NT_Inst ntInst)
{
  this->tag_family = tag16h5_create();
  this->detector = apriltag_detector_create();
  this->ntInst = ntInst;
  apriltag_detector_add_family(detector, tag_family);

  detector->quad_decimate = Config::atag->quad_decimate;
  detector->quad_sigma = Config::atag->blur;
  detector->nthreads = Config::atag->threads;
  detector->debug = Config::atag->debug;
  detector->refine_edges = Config::atag->refine_edges;
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

    /* VERY TEMPORARY, PLEASE REMOVE WHEN NOT NEEDED */
    nt::DeleteAllEntries(ntInst);

    apriltag_detection_t* det;
    zarray_get(detections, i, &det);
    line(img, Point(det->p[0][0], det->p[0][1]), Point(det->p[1][0], det->p[1][1]), Scalar(0, 0xff, 0), 2);
    line(img, Point(det->p[0][0], det->p[0][1]), Point(det->p[3][0], det->p[3][1]), Scalar(0, 0, 0xff), 2);
    line(img, Point(det->p[1][0], det->p[1][1]), Point(det->p[2][0], det->p[2][1]), Scalar(0xff, 0, 0), 2);
    line(img, Point(det->p[2][0], det->p[2][1]), Point(det->p[3][0], det->p[3][1]), Scalar(0xff, 0, 0), 2);

    std::stringstream ss;
    ss << det->id;
    String text = ss.str();

    String rootPath = (std::stringstream() << Config::nt->rootPrefix << "/Detection (" << text << ")/").str();
    NT_Entry e;
    e = nt::GetEntry(ntInst, rootPath + "Corner 0");
    nt::SetEntryTypeValue(e, nt::Value::MakeDoubleArray({det->p[0][0],det->p[0][1]}));
    e = nt::GetEntry(ntInst, rootPath + "Corner 1");
    nt::SetEntryTypeValue(e, nt::Value::MakeDoubleArray({det->p[1][0],det->p[1][1]}));
    e = nt::GetEntry(ntInst, rootPath + "Corner 2");
    nt::SetEntryTypeValue(e, nt::Value::MakeDoubleArray({det->p[2][0],det->p[2][1]}));
    e = nt::GetEntry(ntInst, rootPath + "Corner 3");
    nt::SetEntryTypeValue(e, nt::Value::MakeDoubleArray({det->p[3][0],det->p[3][1]}));

    double fontscale = 1.0;
    int baseline;
    Size textsize = getTextSize(text, FONT, fontscale, 2, &baseline);
    putText(img, text, Point(det->c[0] - textsize.width / 2, det->c[1] + textsize.height / 2), FONT, fontscale,
            Scalar(0xff, 0x99, 0), 2);
  }
  apriltag_detections_destroy(detections);
}