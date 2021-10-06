/*
 Copyright (c) 2020 Neil Puthuff
 Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include <avt_vimba_camera/mono_camera.h>
#include <chrono>
#include <thread>

int main(int argc, char * argv[])
{
  std::cout << "node init" << std::endl;
  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within a launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::Rate(100);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(false);

  std::cout << "make_shared before" << std::endl;
  auto avt_vimba_camera = std::make_shared<avt_vimba_camera::MonoCamera>(
      "avt_vimba_camera", options);

  exec.add_node(avt_vimba_camera);

  exec.spin();

  rclcpp::shutdown();
  return 0;
}