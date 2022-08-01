#include "PSPVideo.h"
#include "MrawReader.h"
#include "gtest/gtest.h"

TEST(MrawTest, MrawHandlesCorrectlyFormattedInputData) {
  upsp::VideoProperties props;
  upsp::MrawReader reader("mraw/12bitMRAW.mraw");
  reader.load_properties(props);
  EXPECT_EQ(props.height, 1024);
  EXPECT_EQ(props.width, 1024);
  EXPECT_EQ(props.bit_depth, 12);
  EXPECT_EQ(props.num_frames, 2);
}
