#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <chrono>
#include <cstring>

// Mock Serial Class
class MockSerial {
public:
    MockSerial() : is_open_(false), baudrate_(9600) {}
    
    bool open() {
        is_open_ = true;
        std::cout << "MockSerial: Port opened successfully" << std::endl;
        return true;
    }
    
    void close() {
        is_open_ = false;
        std::cout << "MockSerial: Port closed" << std::endl;
    }
    
    bool isOpen() const {
        return is_open_;
    }
    
    void setPort(const std::string& port) {
        port_ = port;
        std::cout << "MockSerial: Setting port to " << port << std::endl;
    }
    
    void setBaudrate(unsigned long baudrate) {
        baudrate_ = baudrate;
        std::cout << "MockSerial: Setting baudrate to " << baudrate << std::endl;
    }
    
    size_t write(const std::string& data) {
        if (!is_open_) {
            std::cerr << "MockSerial: Cannot write - port not open" << std::endl;
            return 0;
        }
        
        std::cout << "MockSerial: Writing data: " << data << std::endl;
        // Store the data for later "reading"
        rx_buffer_ += data;
        return data.length();
    }
    
    size_t write(const uint8_t* data, size_t size) {
        if (!is_open_) {
            std::cerr << "MockSerial: Cannot write - port not open" << std::endl;
            return 0;
        }
        
        std::cout << "MockSerial: Writing binary data, " << size << " bytes" << std::endl;
        // For binary data, just append to our buffer
        std::string str_data(reinterpret_cast<const char*>(data), size);
        rx_buffer_ += str_data;
        return size;
    }
    
    std::string read(size_t size) {
        if (!is_open_) {
            std::cerr << "MockSerial: Cannot read - port not open" << std::endl;
            return "";
        }
        
        // Simulate delay
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        // If we have data, return some of it
        if (rx_buffer_.empty()) {
            return "";
        }
        
        size_t bytes_to_read = std::min(size, rx_buffer_.length());
        std::string result = rx_buffer_.substr(0, bytes_to_read);
        rx_buffer_.erase(0, bytes_to_read);
        
        std::cout << "MockSerial: Read " << result.length() << " bytes: " << result << std::endl;
        return result;
    }
    
    size_t read(uint8_t* buffer, size_t size) {
        if (!is_open_) {
            std::cerr << "MockSerial: Cannot read - port not open" << std::endl;
            return 0;
        }
        
        // Simulate delay
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        // If we have data, return some of it
        if (rx_buffer_.empty()) {
            return 0;
        }
        
        size_t bytes_to_read = std::min(size, rx_buffer_.length());
        std::memcpy(buffer, rx_buffer_.c_str(), bytes_to_read);
        rx_buffer_.erase(0, bytes_to_read);
        
        std::cout << "MockSerial: Read " << bytes_to_read << " bytes (binary)" << std::endl;
        return bytes_to_read;
    }
    
    // Simple timeout mock
    struct Timeout {
        static Timeout simpleTimeout(size_t /* ms */) {
            return Timeout();
        }
    };
    
    void setTimeout(const Timeout& /* timeout */) {
        std::cout << "MockSerial: Setting timeout" << std::endl;
    }
    
private:
    bool is_open_;
    std::string port_;
    unsigned long baudrate_;
    std::string rx_buffer_;
};

// Simple function to test our mock serial
void testMockSerial() {
    std::cout << "\n===== Testing Mock Serial =====\n" << std::endl;
    
    MockSerial serial;
    serial.setPort("/dev/mock");
    serial.setBaudrate(9600);
    
    if (serial.open()) {
        std::cout << "Port opened successfully" << std::endl;
        
        // Test write
        std::string test_string = "Hello Mock Serial!\r\n";
        size_t bytes_written = serial.write(test_string);
        std::cout << "Wrote " << bytes_written << " bytes" << std::endl;
        
        // Test read
        std::string response = serial.read(100);
        std::cout << "Read " << response.length() << " bytes: " << response << std::endl;
        
        // Test binary write/read
        uint8_t binary_data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
        bytes_written = serial.write(binary_data, sizeof(binary_data));
        std::cout << "Wrote " << bytes_written << " binary bytes" << std::endl;
        
        uint8_t read_buffer[10] = {0};
        size_t bytes_read = serial.read(read_buffer, sizeof(read_buffer));
        std::cout << "Read " << bytes_read << " binary bytes" << std::endl;
        
        // Close the port
        serial.close();
    } else {
        std::cerr << "Failed to open port" << std::endl;
    }
    
    std::cout << "\n===== Mock Serial Test Complete =====\n" << std::endl;
}

int main() {
    testMockSerial();
    return 0;
} 