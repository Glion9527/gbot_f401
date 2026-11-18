#pragma once

/* 极简测试辅助：不引入 Catch2/GoogleTest，保持宿主骨架零外部依赖。
 * 每个测试文件自己写 main()，失败计数非零时返回 1。 */

#include <cmath>
#include <cstdio>

namespace test {

inline int& failures()
{
    static int count = 0;
    return count;
}

inline void check(bool condition, const char* expression, const char* file, int line)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL %s:%d: %s\n", file, line, expression);
        ++failures();
    }
}

template <typename Left, typename Right>
inline void checkEqual(const Left& left, const Right& right, const char* left_expression,
                       const char* right_expression, const char* file, int line)
{
    if (!(left == right)) {
        std::fprintf(stderr, "FAIL %s:%d: %s == %s\n", file, line, left_expression,
                     right_expression);
        ++failures();
    }
}

inline void checkNear(double left, double right, double epsilon, const char* left_expression,
                      const char* right_expression, const char* file, int line)
{
    if (!(std::fabs(left - right) <= epsilon)) {
        std::fprintf(stderr, "FAIL %s:%d: %s ≈ %s (%.12g vs %.12g, eps %.12g)\n", file, line,
                     left_expression, right_expression, left, right, epsilon);
        ++failures();
    }
}

inline int finish(const char* name)
{
    if (failures() == 0) {
        std::printf("PASS %s\n", name);
        return 0;
    }
    std::fprintf(stderr, "FAILED %s (%d failures)\n", name, failures());
    return 1;
}

}  // namespace test

#define CHECK(condition) ::test::check((condition), #condition, __FILE__, __LINE__)
#define CHECK_EQ(left, right) \
    ::test::checkEqual((left), (right), #left, #right, __FILE__, __LINE__)
#define CHECK_NEAR(left, right, epsilon) \
    ::test::checkNear((left), (right), (epsilon), #left, #right, __FILE__, __LINE__)
