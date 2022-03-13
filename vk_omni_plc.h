#ifndef _VK_OMNI_PLC_H
#define _VK_OMNI_PLC_H

#include "CppLinuxSerial/SerialPort.hpp"
#include "vk_omni_plc_serial.h"

#include <memory>   /*This header defines general ultilities to manage dynamic memory*/
#include <vector>
#include <thread>

#include <ros/ros.h>
#include <std_msgs/UInt8.h>

using namespace mn::CppLinuxSerial;

namespace safety {
    std_msgs::UInt8 msg_(uint8_t data)
    {
      std_msgs::UInt8 msg;
      
      msg.data = data;
      
      return msg;
    }
  
  struct safetyPLCStatus
  {
    int resendCount; /* Biến đếm số lần gửi lại bản tin*/
    
    bool flag; /* Cờ báo quá số lần gửi lại */
    
    std_msgs::UInt8 lastCommand; /* Lưu mã lệnh gần nhất, phục vụ gửi lại */
    
    safetyPLCStatus() : resendCount(0), flag(false) {}
   
    void increaseResendCount() {resendCount++;}
    
    void resetResendCount() {resendCount = 0;}
    
    void flagUp() {flag = true;}
    
    void flagDown() {flag = false;}
  }
  
  class safetyPLC
  {
    private:
            ros::Publisher statusPublisher; /*Tạo một publisher publish topic "robot_status"*/
    
            ros::Subscriber commandSubscriber; /* Tạo một subscriber to subscribe topic "plc_command" */
            
            std::unique_ptr<SerialPort> port; /*serial port -- con trỏ class "SerialPort" chứa các thuộc tính và hàm dùng trong truyền thông nối tiếp*/
            
            int id; /*Mã ID của PLC. Dùng với nhiều PLC slaves*/
            
            std_msgs::UInt8 status; /*status của robot*/
    
            std::unique_ptr<safetyPLCStatus> statusPLC; /*Trạng thái PLC, con trỏ struct "safetyPLCStatus" */
    
            std::thread readThread; /* Tạo Thread hoạt động song song với Main Thread đọc dữ liệu từ cổng RS232 */
    
            bool threadAlive; /* cờ báo trạng thái hoạt động của "readThread" */
            
            int responeRead(); /* Hàm xử lý bản tin đọc về */
    
            void responeHandler(); /* vòng lặp đọc dữ liệu */
    
            void resendHandler(); /* Handle resend request */
    public:
            safetyPLC(ros::NodeHandle *nh);
            
            safetyPLC(ros::NodeHandle *nh, cost std::string& device, int addr);
    
            ~safetyPLC();
    
            void commandSend(std_msgs::UInt8 msg); /* send commands to PLC */
    
            void statusUpdate(uint8_t data); /* update status of robot. Save in "status" */
  };
}

#endif //VK_OMNI_PLC_H

