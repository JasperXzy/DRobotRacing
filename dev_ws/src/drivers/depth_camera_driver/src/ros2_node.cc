#include <sys/sem.h>
#include <sys/shm.h>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "ros2_device.h"
#include "aurora900_ros2_device.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("depth_camera_node");
  RCLCPP_DEBUG(node->get_logger(), "hello depth_camera");
  std::shared_ptr<RosDevice> device(new Aurora900RosDevice(node));

  DeptrumRosDeviceParams param = {};
  param = device->params_;
  int boot_order = param.boot_order;
  if (param.device_numbers > 1) {
    int shmid;
    char* shm = NULL;
    if (boot_order == 1) {
      if ((shmid = shmget((key_t) STELLAR_PID, 1, 0666 | IPC_CREAT)) == -1)
        RCLCPP_ERROR(node->get_logger(), "BOOT_ORDER_%d::Create Share Memory Error:%s", param.boot_order, strerror(errno));
      shm = (char*) shmat(shmid, 0, 0);
      *shm = 1;
      device->InitDevice();
      RCLCPP_DEBUG(node->get_logger(), "BOOT_ORDER_%d::Device %s already open", param.boot_order, param.serial_number.c_str());
      *shm = 2;
    } else {
      if ((shmid = shmget((key_t) STELLAR_PID, 1, 0666 | IPC_CREAT)) == -1)
        RCLCPP_ERROR(node->get_logger(), "BOOT_ORDER_%d::Create Share Memory Error:%s", param.boot_order, strerror(errno));
      shm = (char*) shmat(shmid, 0, 0);

      while (*shm != boot_order)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

      device->InitDevice();
      RCLCPP_DEBUG(node->get_logger(), "BOOT_ORDER_%d::Device %s already open.", param.boot_order, param.serial_number.c_str());
      *shm = boot_order + 1;
    }
    if (boot_order == param.device_numbers) {
      if (shmdt(shm) == -1)
        RCLCPP_ERROR(node->get_logger(), "BOOT_ORDER_%d shmdt failed:%s", param.boot_order, strerror(errno));
      if (shmctl(shmid, IPC_RMID, 0) == -1)
        RCLCPP_ERROR(node->get_logger(), "BOOT_ORDER_%d shmctl(IPC_RMID) failed :%s", param.boot_order, strerror(errno));
    } else {
      if (shmdt(shm) == -1)
        RCLCPP_ERROR(node->get_logger(), "BOOT_ORDER_%d shmdt failed:%s", param.boot_order, strerror(errno));
    }
  } else {
    device->InitDevice();
  }

  device->Start();
  rclcpp::spin(node);
  rclcpp::shutdown();
  // RCLCPP_INFO(node->get_logger(), "process quit succeed"); // Cannot use node logger after shutdown/spin
  std::cout << "process quit succeed" << std::endl;
  return 0;
}
