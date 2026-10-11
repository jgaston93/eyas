#include "util/csv_table.hpp"

#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "util/error.hpp"
#include "util/units.hpp"

namespace eyas {
namespace {

using units::kDegree;
using units::kFoot;

constexpr const char* kTable1D =
    "# source: unit test\n"
    "x[ft],y[deg]\n"
    "0,0\n"
    "1,10\n"
    "3,30\n";

TEST(CsvTable, ParsesHeaderUnitsAndMetadata) {
    const CsvTable csv = parse_csv_table(kTable1D, "t1");
    EXPECT_EQ(csv.source(), "unit test");
    EXPECT_FALSE(csv.is_placeholder());
    ASSERT_EQ(csv.names.size(), 2U);
    EXPECT_EQ(csv.names[0], "x");
    EXPECT_EQ(csv.names[1], "y");
    EXPECT_EQ(csv.column("x")[2], 3.0 * kFoot);
    EXPECT_EQ(csv.column("y")[1], 10.0 * kDegree);
}

TEST(CsvTable, BuildsTable1DInSi) {
    const Table1D t = make_table1d(parse_csv_table(kTable1D, "t1"), "x", "y");
    EXPECT_NEAR(t(2.0 * kFoot), 20.0 * kDegree, 1e-15);
}

TEST(CsvTable, ToleratesWhitespaceBlankLinesAndCrLf) {
    const CsvTable csv = parse_csv_table(
        "# source:  spaced  \r\n\r\n x[m] , v[1] \r\n 0 , 1 \r\n\r\n 1 , 2 \r\n", "t");
    EXPECT_EQ(csv.source(), "spaced");
    EXPECT_EQ(csv.column("v")[1], 2.0);
}

TEST(CsvTable, PlaceholderStatus) {
    const CsvTable csv =
        parse_csv_table("# source: guess\n# status: placeholder\nx[m],v[1]\n0,1\n1,2\n", "t");
    EXPECT_TRUE(csv.is_placeholder());
}

TEST(CsvTable, RequiresSource) {
    EXPECT_THROW(parse_csv_table("x[m],v[1]\n0,1\n1,2\n", "t"), InitError);
    EXPECT_THROW(parse_csv_table("# source:\nx[m],v[1]\n0,1\n1,2\n", "t"), InitError);
}

TEST(CsvTable, RejectsFormatErrors) {
    const std::string src = "# source: t\n";
    EXPECT_THROW(parse_csv_table(src + "x,v[1]\n0,1\n", "t"), InitError);        // no unit
    EXPECT_THROW(parse_csv_table(src + "x[lb],v[1]\n0,1\n", "t"), InitError);    // ambiguous
    EXPECT_THROW(parse_csv_table(src + "x[m],v[1]\n0,1,2\n", "t"), InitError);   // ragged
    EXPECT_THROW(parse_csv_table(src + "x[m],v[1]\n0,abc\n", "t"), InitError);   // not a number
    EXPECT_THROW(parse_csv_table(src + "x[m],v[1]\n0,1.5e\n", "t"), InitError);  // partial
    EXPECT_THROW(parse_csv_table(src + "x[m],v[1]\n0,nan\n", "t"), InitError);   // non-finite
    EXPECT_THROW(parse_csv_table(src + "x[m],v[1]\n0,+-5\n", "t"), InitError);   // two signs
    EXPECT_THROW(parse_csv_table(src + "x[m],x[m]\n0,1\n", "t"), InitError);     // duplicate
    EXPECT_THROW(parse_csv_table(src + "x[m],v[1]\n", "t"), InitError);          // no rows
    EXPECT_THROW(parse_csv_table(src + "# source: again\nx[m]\n0\n", "t"), InitError);
}

TEST(CsvTable, ErrorsNameTheOriginAndLine) {
    try {
        parse_csv_table("# source: t\nx[m],v[1]\n0,1\n0,oops\n", "cl_table.csv");
        FAIL() << "expected InitError";
    } catch (const InitError& e) {
        const std::string what = e.what();
        EXPECT_NE(what.find("cl_table.csv"), std::string::npos) << what;
        EXPECT_NE(what.find("line 4"), std::string::npos) << what;
    }
}

TEST(CsvTable, MissingColumnIsAnError) {
    EXPECT_THROW(make_table1d(parse_csv_table(kTable1D, "t"), "x", "z"), InitError);
}

// A hand-built table whose parallel vectors disagree is an InitError, not an
// out-of-range read.
TEST(CsvTable, MismatchedHandBuiltTableIsAnError) {
    CsvTable csv;
    csv.origin = "hand";
    csv.names = {"x", "v"};
    csv.columns = {{0.0, 1.0}, {2.0, 3.0}};
    EXPECT_THROW(csv.require_dimension("v", units::Dimension::dimensionless), InitError);
    EXPECT_THROW((void)csv.column("v"), InitError);
    csv.dimensions = {units::Dimension::dimensionless, units::Dimension::dimensionless};
    EXPECT_NO_THROW(csv.require_dimension("v", units::Dimension::dimensionless));
    csv.columns.pop_back();
    EXPECT_THROW((void)csv.column("v"), InitError);
}

constexpr const char* kTable2D =
    "# source: unit test\n"
    "a[1],b[1],v[1]\n"
    "1,10,110\n"  // rows deliberately out of order
    "0,10,10\n"
    "0,20,20\n"
    "1,20,120\n"
    "2,10,210\n"
    "2,20,220\n";

TEST(CsvTable, BuildsTable2DFromUnorderedRows) {
    const Table2D t = make_table2d(parse_csv_table(kTable2D, "t2"), "a", "b", "v");
    EXPECT_EQ(t.x_axis().size(), 3U);
    EXPECT_EQ(t.y_axis().size(), 2U);
    EXPECT_EQ(t(1.0, 20.0), 120.0);
    EXPECT_DOUBLE_EQ(t(1.5, 15.0), 165.0);
}

TEST(CsvTable, GridMustBeCompleteAndUnique) {
    const std::string head = "# source: t\na[1],b[1],v[1]\n";
    // Missing (1, 20).
    EXPECT_THROW(
        make_table2d(parse_csv_table(head + "0,10,0\n0,20,0\n1,10,0\n", "t"), "a", "b", "v"),
        InitError);
    // Duplicate (0, 10).
    EXPECT_THROW(
        make_table2d(parse_csv_table(head + "0,10,0\n0,10,1\n0,20,0\n1,10,0\n1,20,0\n", "t"), "a",
                     "b", "v"),
        InitError);
}

TEST(CsvTable, BuildsTable3D) {
    std::string text = "# source: t\nx[1],y[1],z[1],v[1]\n";
    for (int x = 0; x < 2; ++x) {
        for (int y = 0; y < 3; ++y) {
            for (int z = 0; z < 2; ++z) {
                text += std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) +
                        "," + std::to_string(100 * x + 10 * y + z) + "\n";
            }
        }
    }
    const Table3D t = make_table3d(parse_csv_table(text, "t3"), "x", "y", "z", "v");
    EXPECT_EQ(t(1.0, 2.0, 1.0), 121.0);
    EXPECT_DOUBLE_EQ(t(0.5, 1.5, 0.5), 65.5);
}

TEST(CsvTable, ReadsFromFile) {
    const auto path = std::filesystem::path(testing::TempDir()) / "eyas_csv_table_test.csv";
    {
        std::ofstream out(path);
        out << kTable1D;
    }
    const CsvTable csv = read_csv_table(path);
    EXPECT_EQ(csv.column("x").size(), 3U);
    EXPECT_NE(csv.origin.find("eyas_csv_table_test.csv"), std::string::npos);
    std::filesystem::remove(path);
    EXPECT_THROW(read_csv_table(path), InitError);
}

}  // namespace
}  // namespace eyas
