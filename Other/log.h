#define LOGGING
#ifdef LOGGING
#define LOG(s) std::cout << s << std::endl
#define LOG2(s, x) std::cout << s << "\t" << x << std::endl
#define LOG3(s, x, y) std::cout << s << "\t" << x << "\t" << y << std::endl
#elif
#define LOG(x)
#define LOG(x, y)
#define LOG3(s, x, y)
#endif