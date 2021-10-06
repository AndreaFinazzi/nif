#include "avt_vimba_camera/stereo_camera.h"
#include <nodelet/nodelet.h>

namespace avt_vimba_camera {

class StereoCameraNodelet : public nodelet::Nodelet {
public:
  virtual void onInit();
  virtual ~StereoCameraNodelet();

private:
  StereoCamera* camera_;
};

} // namespace avt_vimba_camera
