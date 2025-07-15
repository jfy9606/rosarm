#include <iostream>
#include <string>
#include <serial/serial.h>
#include <thread>
#include <chrono>
#include <vector>

void listAvailablePorts() {
    std::cout << "Available serial ports:" << std::endl;
    std::vector<serial::PortInfo> ports = serial::list_ports();
    
    if (ports.empty()) {
        std::cout << "  No ports available" << std::endl;
    } else {
        for (const auto& port : ports) {
            std::cout << "  " << port.port << " - " << port.description << " (";
            if (port.hardware_id.empty()) {
                std::cout << "No hardware ID";
            } else {
                std::cout << port.hardware_id;
            }
            std::cout << ")" << std::endl;
        }
    }
}

int main(int argc, char** argv) {
    // List available ports
    listAvailablePorts();
    
    std::string port = "/dev/ttyUSB0";
    unsigned long baud = 9600;
    
    if (argc > 1) {
        port = argv[1];
    }
    
    if (argc > 2) {
        baud = std::stoul(argv[2]);
    }
    
    std::cout << "\nAttempting to open serial port: " << port << " at " << baud << " baud" << std::endl;
    
    // Create a serial object
    serial::Serial my_serial;
    
    try {
        // Set up the serial port
        my_serial.setPort(port);
        my_serial.setBaudrate(baud);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        my_serial.setTimeout(timeout);
        
        // Open the serial port
        my_serial.open();
        
        if (my_serial.isOpen()) {
            std::cout << "Serial port opened successfully!" << std::endl;
        } else {
            std::cerr << "Failed to open serial port!" << std::endl;
            return 1;
        }
        
        // Wait for a moment for the serial port to initialize
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Send a simple command
        std::string test_string = "Hello Serial Port!\n";
        size_t bytes_written = my_serial.write(test_string);
        
        std::cout << "Wrote " << bytes_written << " bytes" << std::endl;
        
        // Wait for a response
        std::cout << "Reading response..." << std::endl;
        
        std::string response = my_serial.read(100);
        std::cout << "Read " << response.length() << " bytes: ";
        if (response.length() > 0) {
            std::cout << response << std::endl;
        } else {
            std::cout << "No data received" << std::endl;
        }
        
        // Clean up
        my_serial.close();
        std::cout << "Serial port closed" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 