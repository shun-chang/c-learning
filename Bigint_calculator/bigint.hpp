#ifndef BIGINT_HPP
#define BIGINT_HPP

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <string>

class Bigint final {
public:
  Bigint();
  explicit Bigint(const std::string &raw_str);
  Bigint(const Bigint &rhs);
  Bigint(Bigint &&rhs) noexcept;
  Bigint &operator=(const Bigint &rhs);
  Bigint &operator=(Bigint &&rhs) noexcept;
  Bigint &operator+=(const Bigint &rhs);
  Bigint &operator-=(const Bigint &rhs);
  Bigint &operator*=(const Bigint &rhs);
  Bigint &operator/=(const Bigint &rhs);
  friend Bigint operator+(const Bigint &lhs, const Bigint &rhs);
  friend Bigint operator-(const Bigint &lhs, const Bigint &rhs);
  friend Bigint operator*(const Bigint &lhs, const Bigint &rhs);
  friend Bigint operator/(const Bigint &lhs, const Bigint &rhs);
  std::size_t getDigitsname() const;
  std::string getstr() const;
  explicit operator bool() const noexcept;

private:
  bool is_negative;
  std::string digits;
  bool is_zero() const noexcept;
  static int compare_unsigned(const std::string & a,const std::string &b) noexcept;
  static std::string add_unsigned(const std::string &a,const std::string &b) noexcept;
  static std::string sub_unsigned(const std::string &a,const std::string &b) noexcept;
  static std::string mul_unsigned(const std::string &a,const std::string &b) noexcept;
  static std::string div_unsigned(const std::string &a,const std::string &b,std::string& out_remainder) noexcept;
  static bool is_valid_number_str(const std::string raw_str) noexcept;
  static void process_input_str(const std::string& raw_str, bool& out_is_negative, std::string& out_digits);
  static std::string shift_left_unsigned(const std::string& num, std::size_t n) noexcept;
};
#endif