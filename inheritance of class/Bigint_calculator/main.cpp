#include "bigint.hpp"
#include <string>
#include <iostream>

int main(){
    std::cout << "Please enter the expression (format: Number1 Operator Number2):" << std::endl;
    std::string input;
    std::getline(std::cin,input);
    std::size_t first_space = input.find(' ');
    std::size_t last_space = input.rfind(' ');
    std::string num1_str = input.substr(0,first_space);
    std::string op_str = input.substr(first_space + 1,1);
    std::string num2_str = input.substr(last_space + 1);
    char op=op_str[0];
    Bigint num1(num1_str);
    Bigint num2(num2_str);
    try {
            Bigint result;
            switch (op) {
                case '+':
                    result = num1 + num2;
                    break;
                case '-':
                    result = num1 - num2;
                    break;
                case '*':
                    result = num1 * num2;
                    break;
                case '/':
                    result = num1 / num2;
                    break;
                default:
                    std::cout << "Error: Unknown operator!" << std::endl;
                    break;
            }
            std::cout << "result: " << num1.getstr() << " " << op << " " << num2.getstr() << " = " << result.getstr() << std::endl;
        } catch (const std::domain_error& e) {
            std::cout << "error: " << e.what() << std::endl;
        }
}