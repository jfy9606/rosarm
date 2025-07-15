#ifndef GUI_MOCK_SERVICES_HPP
#define GUI_MOCK_SERVICES_HPP

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <vector>
#include <functional>

namespace gui {

// Mock JointControl service
namespace mock {
  
  // Mock service request
  struct JointControlRequest {
    std::vector<double> position;
  };
  
  // Mock service response
  struct JointControlResponse {
    bool success;
    std::string message;
  };
  
  // Mock VacuumCmd service request
  struct VacuumCmdRequest {
    bool enable;
  };
  
  // Mock VacuumCmd service response
  struct VacuumCmdResponse {
    bool success;
    std::string message;
  };
  
  // Mock client interface
  template<typename RequestT, typename ResponseT>
  class Client {
  public:
    using SharedPtr = std::shared_ptr<Client<RequestT, ResponseT>>;
    using SharedFuture = std::shared_future<std::shared_ptr<ResponseT>>;
    using CallbackType = std::function<void(SharedFuture)>;
    
    Client() {}
    
    bool wait_for_service(std::chrono::duration<double> timeout = std::chrono::duration<double>::max()) {
      return true;
    }
    
    SharedFuture async_send_request(
        std::shared_ptr<RequestT> /* request */,
        CallbackType callback = nullptr) {
      auto response = std::make_shared<ResponseT>();
      response->success = true;
      response->message = "Mock service call succeeded";
      
      std::promise<std::shared_ptr<ResponseT>> promise;
      promise.set_value(response);
      
      auto future = promise.get_future().share();
      
      if (callback) {
        callback(future);
      }
      
      return future;
    }
  };
  
  // Convenience typedefs
  using JointControlClient = Client<JointControlRequest, JointControlResponse>;
  using VacuumCmdClient = Client<VacuumCmdRequest, VacuumCmdResponse>;
}

} // namespace gui

#endif // GUI_MOCK_SERVICES_HPP 