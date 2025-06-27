#include <boost/dll.hpp>
#include <iostream>
#include <filesystem>

int main(int argc, char** argv)
{
    if (argc == 2) {
        boost::dll::library_info inf(argv[1]);
        std::vector<std::string> exports = inf.symbols();
        for (std::size_t j = 0; j < exports.size(); ++j) {
            std::cout << exports[j] << std::endl;
        }
    } else {
        std::cout << "Provide a path to an so" << std::endl;
    }
}