#include "bigint.hpp"
#include <cctype>
#include <stdexcept>

Bigint::Bigint() : is_negative(false),digits("0"){}
Bigint::Bigint(const std::string &raw_str){
    if (!is_valid_number_str(raw_str)) {
        throw std::invalid_argument("Invalid number string: " + raw_str);
    }
    process_input_str(raw_str,is_negative,digits);
}

Bigint::Bigint(const Bigint &rhs): is_negative(rhs.is_negative),digits(rhs.digits){}

Bigint::Bigint(Bigint &&rhs) noexcept : is_negative(rhs.is_negative),digits(rhs.digits){
    rhs.is_negative=false;
    rhs.digits="0";
}

Bigint& Bigint::operator=(const Bigint & rhs){
    if(this!=&rhs){
        this->is_negative=rhs.is_negative;
        this->digits=rhs.digits;
    }
    return *this;
}

Bigint& Bigint::operator=(Bigint&& rhs) noexcept {
    if(this!=&rhs){
        this->is_negative=rhs.is_negative;
        this->digits=std::move(rhs.digits);
        rhs.is_negative=false;
        rhs.digits="0";
    }
    return *this;
}

Bigint& Bigint::operator+=(const Bigint& rhs){
    if(is_zero()){
        *this=rhs;
        return *this;
    }
    if(rhs.is_zero()){
        return *this;
    }
    if(this->is_negative==rhs.is_negative){
        this->digits=add_unsigned(this->digits,rhs.digits);
        return *this;
    }
    int cmp=compare_unsigned(this->digits,rhs.digits);
    if(cmp==0){
        this->is_negative=false;
        this->digits="0";
        return *this;
    }
    if(cmp>0){
        this->digits=sub_unsigned(this->digits,rhs.digits);
        return *this;
    }
    else{
        this->is_negative=rhs.is_negative;
        this->digits=sub_unsigned(rhs.digits,this->digits);
        return *this;
    }
}

Bigint& Bigint::operator-=(const Bigint& rhs){
    if (is_zero()) {
        is_negative = !rhs.is_negative;
        digits = rhs.digits;
        if (rhs.is_zero()) {
            is_negative = false;
            digits = "0";
        }
        return *this;
    }
    if (rhs.is_zero()) {
        return *this;
    }
    if (is_negative != rhs.is_negative) {
        digits = add_unsigned(digits, rhs.digits);
        return *this;
    }
    int cmp=compare_unsigned(this->digits,rhs.digits);
    if(cmp==0){
        this->is_negative=false;
        this->digits="0";
        return *this;
    }
    if(cmp>0){
        this->digits=sub_unsigned(this->digits,rhs.digits);
        return *this;
    }
    else{
        this->is_negative=!this->is_negative;
        this->digits=sub_unsigned(rhs.digits,this->digits);
        return *this;
    }
}

Bigint& Bigint::operator*=(const Bigint& rhs) {
    if (is_zero() || rhs.is_zero()) {
        is_negative=false;
        digits="0";
        return *this;
    }
    is_negative=(is_negative != rhs.is_negative);
    digits=mul_unsigned(digits, rhs.digits);
    return *this;
}

Bigint& Bigint::operator/=(const Bigint& rhs){
    if (rhs.is_zero()) {
        throw std::domain_error("Division by zero");
    }
    if (is_zero()) {
        is_negative = false;
        digits = "0";
        return *this;
    }
    std::string remainder;
    std::string quotient = div_unsigned(digits, rhs.digits, remainder);
    is_negative = (is_negative != rhs.is_negative);
    digits = quotient.empty() ? "0" : quotient;
    return *this;
}

Bigint operator+(const Bigint& lhs,const Bigint& rhs){
    Bigint result=lhs;
    result+=rhs;
    return result;
}

Bigint operator-(const Bigint& lhs,const Bigint& rhs){
    Bigint result=lhs;
    result-=rhs;
    return result;
}

Bigint operator*(const Bigint& lhs,const Bigint& rhs){
    Bigint result=lhs;
    result*=rhs;
    return result;
}

Bigint operator/(const Bigint& lhs,const Bigint& rhs){
    Bigint result=lhs;
    result/=rhs;
    return result;
}

std::size_t Bigint::getDigitsname() const{
    return digits.size();
}

std::string Bigint::getstr() const{
    if(is_zero()){
        return "0";
    }
    std::string trimmed = digits;
    while(trimmed.size() > 1 && trimmed.back() == '0'){
        trimmed.pop_back();
    }
    std::string num_str(trimmed.rbegin(), trimmed.rend());
    return is_negative?('-'+num_str):num_str;
}
//若没有 operator bool()，需手动写 if (!a.is_zero())，代码冗余；
//有了转换运算符，直接用 if (a) 即可，和 int/double 等基本类型的使用方式一致。
Bigint::operator bool() const noexcept {
    return !is_zero();
}

bool Bigint::is_zero() const noexcept{
    return digits=="0";
}

int Bigint::compare_unsigned(const std::string &a,const std::string &b) noexcept{
    if(a.size()!=b.size()){
        return a.size()>b.size() ? 1 : -1;
    }
    for(auto it_a=a.rbegin(),it_b=b.rbegin();it_a!=a.rend();it_a++,it_b++){
        if(*it_a>*it_b) return 1;
        if(*it_a<*it_b) return -1;
    }
    return 0;
}

std::string Bigint::shift_left_unsigned(const std::string &num, std::size_t n) noexcept{
    if(num =="0"||n==0){
        return num;
    }
    return std::string(n,'0')+num;
}

std::string Bigint::add_unsigned(const std::string&  a,const std::string &b) noexcept{
    int carry=0;
    const std::size_t max_len=std::max(a.size(),b.size());
    std::string result;
    for(auto i=0;i<max_len;i++){
        const int digits_a=(i<a.size()) ? a[i]-'0' : 0;
        const int digits_b=(i<b.size()) ? b[i]-'0' : 0;
        const int sum=digits_a+digits_b+carry;
        result.push_back((sum%10)+'0');
        carry=sum/10;
    }
    if(carry>0){
        result.push_back(carry+'0');
    }
    return result;
}

std::string Bigint::sub_unsigned(const std::string &a,const std::string &b)noexcept{
    int borrow=0;
    std::string result;
    for(auto i=0;i<a.size();i++){
        int digits_a=a[i]-'0'+borrow;
        const int digits_b=(i<b.size())? b[i]-'0' : 0;
        borrow=(digits_a<digits_b) ? -1:0;
        if(digits_a<digits_b){
            digits_a=digits_a+10;
        }
        const int sub=digits_a-digits_b;
        result.push_back(sub+'0');
    }
    while(result.size()>1&&result.back()=='0'){
        result.pop_back();
    }
    return result;
}

std::string Bigint::mul_unsigned(const std::string &a,const std::string &b)noexcept{
    std::string result="0";
    for(auto i=0;i<b.size();i++){
        const int digits_b=b[i]-'0';
        if(digits_b==0) continue;
        std::string temp;
        int carry=0;
        for(auto c:a){
            const int digits_c=c-'0';
            const int product=digits_c*digits_b+carry;
            temp.push_back((product%10)+'0');
            carry=product/10;
        }
        if(carry>0){
            temp.push_back(carry+'0');
        }
        temp=shift_left_unsigned(temp,i);
        result=add_unsigned(result,temp);
    }
    return result;
}

std::string Bigint::div_unsigned(const std::string &a,const std::string &b,std::string &out_remainder) noexcept{
    out_remainder="0";
    std::string quotient;
    if(compare_unsigned(a,b)<0){
        quotient="0";
        out_remainder=a;
    }
    std::string dividend_segment;
    for(auto it=a.rbegin();it!=a.rend();++it){
        std::string temp_remainder(out_remainder.rbegin(), out_remainder.rend());
        temp_remainder.push_back(*it);
        out_remainder.assign(temp_remainder.rbegin(),temp_remainder.rend());
        while (out_remainder.size() > 1 && out_remainder.back() == '0') {
            out_remainder.pop_back();
        }
        int count=0;
        std::string temp=b;
        while(compare_unsigned(out_remainder,temp)>=0){
            out_remainder=sub_unsigned(out_remainder,temp);
            count++;
        }
        quotient.push_back(count+'0');
    }
    std::reverse(quotient.rbegin(),quotient.rend());
    const std::size_t first_non_zero=quotient.find_first_not_of("0");
    if(first_non_zero!=std::string::npos){
        quotient=quotient.substr(first_non_zero);
    }
    else{
        quotient="0";
    }
    return quotient;
}

bool Bigint::is_valid_number_str(const std::string raw_str) noexcept{
    if(raw_str.empty()){
        return false;
    }
    std::size_t start_idx=0;
    if(raw_str[0]=='-'){
        if(raw_str.size()==1){
            return false;
        }
        start_idx=1;
    }
    bool has_non_zero = false;
    for (std::size_t i = start_idx; i < raw_str.size(); ++i) {
        if (!std::isdigit(raw_str[i])) {
            return false;
        }
        if (raw_str[i] != '0') {
            has_non_zero = true;
        }
    }
    return has_non_zero || (raw_str == "0" || raw_str == "-0"); 
}

void Bigint::process_input_str(const std::string &raw_str,bool &out_is_negative,std::string &out_digits){
    out_is_negative=false;
    std::string new_part=raw_str;
    if(new_part[0]=='-'){
        out_is_negative=true;
        new_part=raw_str.substr(1);
    }
    std::size_t first_non_zero=new_part.find_first_not_of('0');
    if(first_non_zero!=std::string::npos){
        new_part=new_part.substr(first_non_zero);
    }
    else{
        new_part="0";
    }
    out_digits.assign(new_part.rbegin(),new_part.rend());
}
