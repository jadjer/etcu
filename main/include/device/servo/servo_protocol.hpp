//
// Created by jadjer on 6.09.26.
//

#pragma once

namespace device {

template <class Driver, class PowerEnable, std::uint8_t ServoId>
  requires concepts::UART<Driver> && concepts::GPIO<PowerEnable>
class ServoProtocol {
  static constexpr std::uint16_t timeout_ms{30};

  Driver& m_driver_uart;
  PowerEnable& m_driver_power;

 public:
  constexpr explicit ServoProtocol(Driver& driver_uart, PowerEnable& driver_power) noexcept : m_driver_uart(driver_uart), m_driver_power(driver_power) {}

  constexpr ServoProtocol() noexcept = delete;

  ServoProtocol(ServoProtocol const&) noexcept = delete;
  auto operator=(ServoProtocol const&) noexcept -> ServoProtocol& = delete;

  ServoProtocol(ServoProtocol&&) noexcept = delete;
  auto operator=(ServoProtocol&&) noexcept -> ServoProtocol& = delete;

  constexpr ~ServoProtocol() noexcept = default;

  [[nodiscard]] auto init_hardware() noexcept -> bool {
    if (!m_driver_uart.init()) [[unlikely]] {
      return false;
    }

    if (!m_driver_power.init()) [[unlikely]] {
      return false;
    }

    return m_driver_power.enable();
  }

  template <std::size_t ParamSize>
  auto send_packet(ServoInstruction const instruction, std::array<std::uint8_t, ParamSize> const& parameters) const noexcept -> void {
    ServoMessage<ParamSize> const message{ServoId, instruction, parameters};

    m_driver_uart.flush();
    m_driver_uart.write(message.to_array());
  }

  template <std::size_t PayloadSize>
  [[nodiscard]] auto receive_packet(ServoMessage<PayloadSize>& message) noexcept -> bool {
    static constexpr std::size_t total_package_size = ServoMessage<PayloadSize>::total_size;

    std::array<std::uint8_t, total_package_size> response_bytes{};

    if (!m_driver_uart.read(response_bytes, timeout_ms)) [[unlikely]] {
      return false;
    }

    auto const response_message = ServoMessage<PayloadSize>{ServoId, response_bytes};

    if (!response_message.is_valid()) [[unlikely]] {
      return false;
    }

    message = response_message;

    return true;
  }
};

}  // namespace device
