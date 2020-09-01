#include <iostream>

int main()
{
    unsigned short x{ 65535 }; // largest 16-bit unsigned value possible
    std::cout << "x was: " << x << '\n';

    x++;
    std::cout << "x is now: " << x << '\n';

    x++;
    std::cout << "x is now: " << x << '\n';

    return 0;
}

