#include "gtest/gtest.h"
#include "../include/plot3d.h"
#include "test_grid_utils.h"

TEST(Plot3dTest, ReadFunctionFileWithRecordSeparators) {
    std::string filename = "sample_data/26-scalars-with-seps.f";
    {  // should work when we ask it to guess for presence of seps based on file contents
        const auto s = upsp::read_plot3d_scalar_function_file(filename);
        EXPECT_EQ(s.size(), 26);
    }
    {  // should work when we explicitly specify record_seps = +1
        const auto s = upsp::read_plot3d_scalar_function_file(filename, +1);
        EXPECT_EQ(s.size(), 26);
    }
    // should fail when we explicitly specify the file HAS record seps
    EXPECT_THROW(upsp::read_plot3d_scalar_function_file(filename, 0), std::invalid_argument);
}

TEST(Plot3dTest, ReadFunctionFileWithoutRecordSeparators) {
    std::string filename = "sample_data/26-scalars-without-seps.f";
    {
        const auto s = upsp::read_plot3d_scalar_function_file(filename);
        EXPECT_EQ(s.size(), 26);
    }
    {
        const auto s = upsp::read_plot3d_scalar_function_file(filename, 0);
        EXPECT_EQ(s.size(), 26);
    }
    // should fail when we explicitly specify the file HAS record seps
    EXPECT_THROW(upsp::read_plot3d_scalar_function_file(filename, +1), std::invalid_argument);
}

/** Test Reading/Writing Structured Plot3D Files */
TEST(Plot3dTest, ReadWriteGridFiles) {

    /********************************************/
    /* Test Single Zone Plot3D Files            */
    /********************************************/
    std::string single_le_sp = "inputs/sphere_unf_single_integration_sp.x";
    std::string single_le_dp = "inputs/sphere_unf_single_integration_dp.x";
    std::string single_be_sp = "inputs/sphere_unf_single_integration_sp_bigend.x";
    std::string single_be_dp = "inputs/sphere_unf_single_integration_dp_bigend.x";

    std::string o_single_le_sp = "outputs/sphere_unf_single_integration_sp_o.x";
    std::string o_single_le_dp = "outputs/sphere_unf_single_integration_dp_o.x";

    upsp::StructuredGrid<float> fgrid;
    upsp::StructuredGrid<double> dgrid;

    // Test single precision
    upsp::read_plot3d_grid_file(single_le_sp, fgrid);
    upsp::write_plot3d_grid_file(o_single_le_sp, fgrid);

    EXPECT_TRUE(compare_files(single_le_sp, o_single_le_sp));
   
    upsp::read_plot3d_grid_file(single_be_sp, fgrid);
    upsp::write_plot3d_grid_file(o_single_le_sp, fgrid);

    EXPECT_TRUE(compare_files(single_le_sp, o_single_le_sp)); 

    // Test double precision
    upsp::read_plot3d_grid_file(single_le_dp, dgrid);
    upsp::write_plot3d_grid_file(o_single_le_dp, dgrid);

    EXPECT_TRUE(compare_files(single_le_dp, o_single_le_dp));
   
    upsp::read_plot3d_grid_file(single_be_dp, dgrid);
    upsp::write_plot3d_grid_file(o_single_le_dp, dgrid);

    EXPECT_TRUE(compare_files(single_le_dp, o_single_le_dp)); 

    // Test single -> double 
    upsp::read_plot3d_grid_file(single_le_sp, dgrid);
    upsp::write_plot3d_grid_file(o_single_le_dp, dgrid);

    EXPECT_TRUE(compare_files(single_le_dp, o_single_le_dp));
   
    // Test double -> single
    upsp::read_plot3d_grid_file(single_le_dp, fgrid);
    upsp::write_plot3d_grid_file(o_single_le_sp, fgrid);

    EXPECT_TRUE(compare_files(single_le_sp, o_single_le_sp));
   
    /********************************************/
    /* Test Multiple Zone Plot3D Files          */
    /********************************************/
    std::string multi_le_sp = "inputs/sphere_unf_multi_integration_sp.x";
    std::string multi_le_dp = "inputs/sphere_unf_multi_integration_dp.x";
    std::string multi_be_sp = "inputs/sphere_unf_multi_integration_sp_bigend.x";
    std::string multi_be_dp = "inputs/sphere_unf_multi_integration_dp_bigend.x";
    std::string multi_ib_sp = "inputs/sphere_unf_multi_integration_sp_iblank.x";
    std::string multi_ib_dp = "inputs/sphere_unf_multi_integration_dp_iblank.x";

    std::string o_multi_le_sp = "outputs/sphere_unf_multi_integration_sp_o.x";
    std::string o_multi_le_dp = "outputs/sphere_unf_multi_integration_dp_o.x";

    // Test single precision
    upsp::read_plot3d_grid_file(multi_le_sp, fgrid);
    upsp::write_plot3d_grid_file(o_multi_le_sp, fgrid);

    EXPECT_TRUE(compare_files(multi_le_sp, o_multi_le_sp));
   
    upsp::read_plot3d_grid_file(multi_be_sp, fgrid);
    upsp::write_plot3d_grid_file(o_multi_le_sp, fgrid);

    EXPECT_TRUE(compare_files(multi_le_sp, o_multi_le_sp)); 

    upsp::read_plot3d_grid_file(multi_ib_sp, fgrid);
    upsp::write_plot3d_grid_file(o_multi_le_sp, fgrid);

    EXPECT_TRUE(compare_files(multi_le_sp, o_multi_le_sp));

    // Test double precision
    upsp::read_plot3d_grid_file(multi_le_dp, dgrid);
    upsp::write_plot3d_grid_file(o_multi_le_dp, dgrid);

    EXPECT_TRUE(compare_files(multi_le_dp, o_multi_le_dp));
   
    upsp::read_plot3d_grid_file(multi_be_dp, dgrid);
    upsp::write_plot3d_grid_file(o_multi_le_dp, dgrid);

    EXPECT_TRUE(compare_files(multi_le_dp, o_multi_le_dp)); 

    upsp::read_plot3d_grid_file(multi_ib_dp, dgrid);
    upsp::write_plot3d_grid_file(o_multi_le_dp, dgrid);

    EXPECT_TRUE(compare_files(multi_le_dp, o_multi_le_dp));
}
