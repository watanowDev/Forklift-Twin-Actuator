// ============================================================================
// usb_device_info.cpp
// Patlite USB 장치 정보 수집 프로그램
//
// 컴파일: g++ -o usb_device_info usb_device_info.cpp -lusb-1.0
// 실행: sudo ./usb_device_info
// ============================================================================

#include <libusb-1.0/libusb.h>
#include <iostream>
#include <iomanip>
#include <vector>

// Patlite Vendor ID
#define PATLITE_VENDOR_ID 0x191a

struct DeviceInfo
{
    uint16_t vendor_id;
    uint16_t product_id;
    uint8_t bus;
    uint8_t address;
    std::string manufacturer;
    std::string product;
    std::string serial;
};

void print_header(const std::string &title)
{
    std::cout << "\n========================================" << std::endl;
    std::cout << title << std::endl;
    std::cout << "========================================" << std::endl;
}

void print_device_info(libusb_device *device, libusb_device_handle *handle)
{
    struct libusb_device_descriptor desc;
    int ret = libusb_get_device_descriptor(device, &desc);
    if (ret < 0)
    {
        std::cerr << "장치 정보 가져오기 실패: " << libusb_error_name(ret) << std::endl;
        return;
    }

    uint8_t bus = libusb_get_bus_number(device);
    uint8_t address = libusb_get_device_address(device);

    std::cout << "\n장치 발견!" << std::endl;
    std::cout << "  Bus: " << static_cast<int>(bus) << std::endl;
    std::cout << "  Address: " << static_cast<int>(address) << std::endl;
    std::cout << "  Vendor ID: 0x" << std::hex << std::setw(4) << std::setfill('0')
              << desc.idVendor << std::dec << std::endl;
    std::cout << "  Product ID: 0x" << std::hex << std::setw(4) << std::setfill('0')
              << desc.idProduct << std::dec << std::endl;
    std::cout << "  Class: 0x" << std::hex << static_cast<int>(desc.bDeviceClass) << std::dec << std::endl;
    std::cout << "  SubClass: 0x" << std::hex << static_cast<int>(desc.bDeviceSubClass) << std::dec << std::endl;
    std::cout << "  Protocol: 0x" << std::hex << static_cast<int>(desc.bDeviceProtocol) << std::dec << std::endl;

    if (handle)
    {
        // 문자열 정보 가져오기
        char string[256];

        if (desc.iManufacturer > 0)
        {
            ret = libusb_get_string_descriptor_ascii(handle, desc.iManufacturer,
                                                     (unsigned char *)string, sizeof(string));
            if (ret > 0)
            {
                std::cout << "  Manufacturer: " << string << std::endl;
            }
        }

        if (desc.iProduct > 0)
        {
            ret = libusb_get_string_descriptor_ascii(handle, desc.iProduct,
                                                     (unsigned char *)string, sizeof(string));
            if (ret > 0)
            {
                std::cout << "  Product: " << string << std::endl;
            }
        }

        if (desc.iSerialNumber > 0)
        {
            ret = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber,
                                                     (unsigned char *)string, sizeof(string));
            if (ret > 0)
            {
                std::cout << "  Serial: " << string << std::endl;
            }
        }
    }
}

void print_config_info(libusb_device *device, libusb_device_handle *handle)
{
    struct libusb_config_descriptor *config;
    int ret = libusb_get_active_config_descriptor(device, &config);
    if (ret < 0)
    {
        std::cerr << "설정 정보 가져오기 실패: " << libusb_error_name(ret) << std::endl;
        return;
    }

    std::cout << "\n설정 정보:" << std::endl;
    std::cout << "  인터페이스 개수: " << static_cast<int>(config->bNumInterfaces) << std::endl;

    for (int i = 0; i < config->bNumInterfaces; i++)
    {
        const struct libusb_interface *interface = &config->interface[i];
        std::cout << "\n  인터페이스 #" << i << ":" << std::endl;

        for (int j = 0; j < interface->num_altsetting; j++)
        {
            const struct libusb_interface_descriptor *iface = &interface->altsetting[j];

            std::cout << "    Alt Setting #" << j << ":" << std::endl;
            std::cout << "      Class: 0x" << std::hex << static_cast<int>(iface->bInterfaceClass) << std::dec;

            // 클래스 이름 출력
            switch (iface->bInterfaceClass)
            {
            case LIBUSB_CLASS_HID:
                std::cout << " (HID)" << std::endl;
                break;
            case LIBUSB_CLASS_VENDOR_SPEC:
                std::cout << " (Vendor Specific)" << std::endl;
                break;
            default:
                std::cout << std::endl;
                break;
            }

            std::cout << "      SubClass: 0x" << std::hex << static_cast<int>(iface->bInterfaceSubClass) << std::dec << std::endl;
            std::cout << "      Protocol: 0x" << std::hex << static_cast<int>(iface->bInterfaceProtocol) << std::dec << std::endl;
            std::cout << "      Endpoint 개수: " << static_cast<int>(iface->bNumEndpoints) << std::endl;

            // Endpoint 정보
            for (int k = 0; k < iface->bNumEndpoints; k++)
            {
                const struct libusb_endpoint_descriptor *ep = &iface->endpoint[k];

                std::cout << "\n      Endpoint #" << k << ":" << std::endl;
                std::cout << "        Address: 0x" << std::hex << static_cast<int>(ep->bEndpointAddress) << std::dec;

                if (ep->bEndpointAddress & LIBUSB_ENDPOINT_IN)
                {
                    std::cout << " (IN)" << std::endl;
                }
                else
                {
                    std::cout << " (OUT)" << std::endl;
                }

                std::cout << "        Type: ";
                switch (ep->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK)
                {
                case LIBUSB_TRANSFER_TYPE_CONTROL:
                    std::cout << "Control" << std::endl;
                    break;
                case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
                    std::cout << "Isochronous" << std::endl;
                    break;
                case LIBUSB_TRANSFER_TYPE_BULK:
                    std::cout << "Bulk" << std::endl;
                    break;
                case LIBUSB_TRANSFER_TYPE_INTERRUPT:
                    std::cout << "Interrupt" << std::endl;
                    break;
                }

                std::cout << "        Max Packet Size: " << ep->wMaxPacketSize << " bytes" << std::endl;
                std::cout << "        Interval: " << static_cast<int>(ep->bInterval) << std::endl;
            }
        }
    }

    libusb_free_config_descriptor(config);
}

void scan_all_usb_devices()
{
    print_header("모든 USB 장치 검색");

    libusb_context *ctx = nullptr;
    int ret = libusb_init(&ctx);
    if (ret < 0)
    {
        std::cerr << "libusb 초기화 실패: " << libusb_error_name(ret) << std::endl;
        return;
    }

    libusb_device **devs;
    ssize_t cnt = libusb_get_device_list(ctx, &devs);
    if (cnt < 0)
    {
        std::cerr << "장치 목록 가져오기 실패: " << libusb_error_name(cnt) << std::endl;
        libusb_exit(ctx);
        return;
    }

    std::cout << "총 " << cnt << "개의 USB 장치 발견" << std::endl;

    for (ssize_t i = 0; i < cnt; i++)
    {
        libusb_device *device = devs[i];
        struct libusb_device_descriptor desc;

        ret = libusb_get_device_descriptor(device, &desc);
        if (ret < 0)
            continue;

        uint8_t bus = libusb_get_bus_number(device);
        uint8_t address = libusb_get_device_address(device);

        std::cout << "\n[" << i + 1 << "] "
                  << "Bus " << std::setw(3) << static_cast<int>(bus) << " "
                  << "Device " << std::setw(3) << static_cast<int>(address) << ": "
                  << "ID " << std::hex << std::setw(4) << std::setfill('0') << desc.idVendor
                  << ":" << std::setw(4) << std::setfill('0') << desc.idProduct << std::dec;

        // Patlite 장치인지 확인
        if (desc.idVendor == PATLITE_VENDOR_ID)
        {
            std::cout << " ★ PATLITE 장치!" << std::endl;
        }
        else
        {
            std::cout << std::endl;
        }
    }

    libusb_free_device_list(devs, 1);
    libusb_exit(ctx);
}

void inspect_patlite_device()
{
    print_header("Patlite 장치 상세 정보");

    libusb_context *ctx = nullptr;
    int ret = libusb_init(&ctx);
    if (ret < 0)
    {
        std::cerr << "libusb 초기화 실패: " << libusb_error_name(ret) << std::endl;
        return;
    }

    // Patlite 장치 찾기 (Product ID는 0으로 설정하여 모든 Patlite 장치 검색)
    libusb_device **devs;
    ssize_t cnt = libusb_get_device_list(ctx, &devs);
    if (cnt < 0)
    {
        std::cerr << "장치 목록 가져오기 실패" << std::endl;
        libusb_exit(ctx);
        return;
    }

    libusb_device *patlite_device = nullptr;
    for (ssize_t i = 0; i < cnt; i++)
    {
        struct libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(devs[i], &desc) == 0)
        {
            if (desc.idVendor == PATLITE_VENDOR_ID)
            {
                patlite_device = devs[i];
                break;
            }
        }
    }

    if (!patlite_device)
    {
        std::cerr << "Patlite 장치를 찾을 수 없습니다" << std::endl;
        std::cerr << "Vendor ID 0x191a 장치가 연결되어 있는지 확인하세요" << std::endl;
        libusb_free_device_list(devs, 1);
        libusb_exit(ctx);
        return;
    }

    libusb_device_handle *handle = nullptr;
    ret = libusb_open(patlite_device, &handle);
    if (ret < 0)
    {
        std::cerr << "장치 열기 실패: " << libusb_error_name(ret) << std::endl;
        std::cerr << "sudo로 실행하거나 udev 규칙을 설정하세요" << std::endl;
        libusb_free_device_list(devs, 1);
        libusb_exit(ctx);
        return;
    }

    // 장치 정보 출력
    print_device_info(patlite_device, handle);

    // 설정 정보 출력
    print_config_info(patlite_device, handle);

    // 정리
    libusb_close(handle);
    libusb_free_device_list(devs, 1);
    libusb_exit(ctx);
}

void print_usage()
{
    std::cout << "\n사용법: usb_device_info [command]\n"
              << std::endl;
    std::cout << "Commands:" << std::endl;
    std::cout << "  scan       모든 USB 장치 검색" << std::endl;
    std::cout << "  patlite    Patlite 장치 상세 정보" << std::endl;
    std::cout << "  help       도움말 출력" << std::endl;
    std::cout << "\n옵션 없이 실행하면 모든 정보를 출력합니다." << std::endl;
    std::cout << "\n주의: USB 장치 접근을 위해 sudo 권한이 필요할 수 있습니다." << std::endl;
}

int main(int argc, char **argv)
{
    std::cout << "========================================" << std::endl;
    std::cout << "Patlite USB 장치 정보 수집 도구" << std::endl;
    std::cout << "========================================" << std::endl;

    if (argc > 1)
    {
        std::string command = argv[1];

        if (command == "scan")
        {
            scan_all_usb_devices();
        }
        else if (command == "patlite")
        {
            inspect_patlite_device();
        }
        else if (command == "help" || command == "-h")
        {
            print_usage();
        }
        else
        {
            std::cerr << "알 수 없는 명령어: " << command << std::endl;
            print_usage();
            return 1;
        }
    }
    else
    {
        // 모든 정보 출력
        scan_all_usb_devices();
        inspect_patlite_device();
    }

    std::cout << "\n========================================" << std::endl;
    std::cout << "완료!" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
