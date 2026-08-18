// Copyright 2026 Pavel Suprunov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Created by jadjer on 17.08.26.
//

#pragma once

namespace commons {

template <typename T, T MinVal, T MaxVal, typename Tag>
class BoundedValue {
 public:
  T m_value{MinVal};

  // Универсальный clamp, защищающий беззнаковые типы от отрицательных чисел на входе
  static constexpr T clamp(int64_t const value) noexcept {
    return value < static_cast<int64_t>(MinVal) ? MinVal
         : value > static_cast<int64_t>(MaxVal) ? MaxVal : static_cast<T>(value);
  }

  constexpr BoundedValue() noexcept = default;

  // Конструктор из сырого числа
  constexpr BoundedValue(int64_t const val) noexcept : m_value(clamp(val)) {} // NOLINT(google-explicit-constructor)

  // Запрещаем неявное конструирование из BoundedValue с ДРУГИМ тегом
  template <typename OtherT, OtherT OMin, OtherT OMax, typename OtherTag>
    requires (!std::same_as<Tag, OtherTag>)
  BoundedValue(BoundedValue<OtherT, OMin, OMax, OtherTag> const&) = delete; // NOLINT(google-explicit-constructor)

  [[nodiscard]] constexpr T get() const noexcept { return m_value; }

  // Сравнение разрешено только с объектами ТОГО ЖЕ типа (с тем же тегом)
  auto operator<=> (const BoundedValue&) const = default;

  // ==================== ОПЕРАТОРЫ ВЫЧИТАНИЯ ====================

  // Разница по модулю между двумя одинаковыми BoundedValue
  constexpr int64_t operator-(BoundedValue const& other) const noexcept {
    return m_value >= other.m_value ? m_value - other.m_value : other.m_value - m_value;
  }

  // Вычитание встроенного числа из BoundedValue (например, pos - 10)
  constexpr int64_t operator-(std::integral auto const other) const noexcept {
    return static_cast<int64_t>(m_value) - static_cast<int64_t>(other);
  }

  // Вычитание BoundedValue из встроенного числа (например, 100 - pos)
  friend constexpr int64_t operator-(std::integral auto const lhs, BoundedValue const& rhs) noexcept {
    return static_cast<int64_t>(lhs) - static_cast<int64_t>(rhs.m_value);
  }

  // ==================== ОПЕРАТОРЫ УМНОЖЕНИЯ ====================

  // Умножение двух одинаковых BoundedValue (например, in * in)
  constexpr int64_t operator*(BoundedValue const& other) const noexcept {
    return static_cast<int64_t>(m_value) * static_cast<int64_t>(other.m_value);
  }

  // Умножение BoundedValue на встроенное число (например, pos * 2)
  constexpr int64_t operator*(std::integral auto const other) const noexcept {
    return static_cast<int64_t>(m_value) * static_cast<int64_t>(other);
  }

  // Умножение встроенного числа на BoundedValue (например, 2 * pos или кубическая часть: (in * in) * in)
  friend constexpr int64_t operator*(std::integral auto const lhs, BoundedValue const& rhs) noexcept {
    return static_cast<int64_t>(lhs) * static_cast<int64_t>(rhs.m_value);
  }

  // Объект + Число (например, servo_pos + 50)
  constexpr int64_t operator+(std::integral auto const other) const noexcept {
    return static_cast<int64_t>(m_value) + static_cast<int64_t>(other);
  }

  // Число + Объект (например, 50 + servo_pos)
  friend constexpr int64_t operator+(std::integral auto const lhs, BoundedValue const& rhs) noexcept {
    return static_cast<int64_t>(lhs) + static_cast<int64_t>(rhs.m_value);
  }

  // ==================== ОПЕРАТОРЫ СРАВНЕНИЯ С ЧИСЛАМИ ====================

  constexpr bool operator<=(std::integral auto const other) const noexcept { return static_cast<int64_t>(m_value) <= static_cast<int64_t>(other); }
  constexpr bool operator>=(std::integral auto const other) const noexcept { return static_cast<int64_t>(m_value) >= static_cast<int64_t>(other); }
  constexpr bool operator>(std::integral auto const other) const noexcept { return static_cast<int64_t>(m_value) > static_cast<int64_t>(other); }
  constexpr bool operator<(std::integral auto const other) const noexcept { return static_cast<int64_t>(m_value) < static_cast<int64_t>(other); }

  friend constexpr bool operator<=(std::integral auto const lhs, BoundedValue const& rhs) noexcept { return static_cast<int64_t>(lhs) <= static_cast<int64_t>(rhs.m_value); }
  friend constexpr bool operator>=(std::integral auto const lhs, BoundedValue const& rhs) noexcept { return static_cast<int64_t>(lhs) >= static_cast<int64_t>(rhs.m_value); }
  friend constexpr bool operator>(std::integral auto const lhs, BoundedValue const& rhs) noexcept { return static_cast<int64_t>(lhs) > static_cast<int64_t>(rhs.m_value); }
  friend constexpr bool operator<(std::integral auto const lhs, BoundedValue const& rhs) noexcept { return static_cast<int64_t>(lhs) < static_cast<int64_t>(rhs.m_value); }
};

}  // namespace commons
