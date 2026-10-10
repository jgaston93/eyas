#include "sim/recorder.hpp"

#include <array>
#include <charconv>
#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "support/alloc_counter.hpp"
#include "util/error.hpp"

namespace eyas {
namespace {

TEST(Recorder, RejectsBadConstruction) {
    EXPECT_THROW(Recorder({}, 10), InitError);
    EXPECT_THROW(Recorder({"a", "a"}, 10), InitError);
    EXPECT_THROW(Recorder({"a"}, 0), InitError);
}

TEST(Recorder, RecordsInOrder) {
    Recorder r({"u", "v"}, 4);
    const std::array<double, 2> s0{1.0, 2.0};
    const std::array<double, 2> s1{3.0, 4.0};
    EXPECT_TRUE(r.record(0, s0));
    EXPECT_TRUE(r.record(1'000'000, s1));
    ASSERT_EQ(r.size(), 2U);
    EXPECT_FALSE(r.wrapped());
    EXPECT_EQ(r.time_ns(1), 1'000'000);
    EXPECT_EQ(r.value(1, 0), 3.0);
    EXPECT_EQ(r.value(0, 1), 2.0);
}

TEST(Recorder, RingKeepsTheNewestSamples) {
    Recorder r({"u"}, 3);
    for (int i = 0; i < 7; ++i) {
        const std::array<double, 1> s{static_cast<double>(i)};
        r.record(i, s);
    }
    ASSERT_EQ(r.size(), 3U);
    EXPECT_TRUE(r.wrapped());
    EXPECT_EQ(r.value(0, 0), 4.0);
    EXPECT_EQ(r.value(2, 0), 6.0);
    EXPECT_EQ(r.time_ns(0), 4);
}

TEST(Recorder, RejectsWrongChannelCount) {
    Recorder r({"u", "v"}, 4);
    const std::array<double, 3> bad{1.0, 2.0, 3.0};
    EXPECT_FALSE(r.record(0, bad));
    EXPECT_EQ(r.size(), 0U);
    EXPECT_EQ(r.rejected(), 1U);
}

TEST(Recorder, RecordDoesNotAllocate) {
    Recorder r({"a", "b", "c"}, 16);
    std::array<double, 3> s{0.1, 0.2, 0.3};
    test::AllocationCounter counter;
    for (int i = 0; i < 100; ++i) {
        s[0] += 1.0;
        r.record(i, s);
    }
    EXPECT_EQ(counter.count(), 0U);
}

TEST(Recorder, CsvRoundTripsExactly) {
    Recorder r({"x", "y"}, 8);
    const std::array<double, 2> s{0.1, -1.0 / 3.0};
    r.record(123'456'789, s);
    const auto path = std::filesystem::path(testing::TempDir()) / "eyas_recorder_test.csv";
    r.write_csv(path);

    std::ifstream in(path);
    std::string header;
    std::string row;
    std::getline(in, header);
    std::getline(in, row);
    EXPECT_EQ(header, "time_ns,x,y");
    const auto c1 = row.find(',');
    const auto c2 = row.find(',', c1 + 1);
    EXPECT_EQ(row.substr(0, c1), "123456789");
    double y = 0.0;
    std::from_chars(row.data() + c2 + 1, row.data() + row.size(), y);
    EXPECT_EQ(y, -1.0 / 3.0);
    std::filesystem::remove(path);
}

}  // namespace
}  // namespace eyas
