#include "board_map.h"

/* X轴5个离散坐标（从上到下） */
const int32_t board_x_step[BOARD_X_POINT_COUNT] = {
  5971,  /* X[0] 待命区最上列 */
  4772,  /* X[1] 待命区第二列,博弈区上列 */
  3552,  /* X[2] 中心列 */
  2409,  /* X[3] 待命区第4列,博弈区下列 */
  1345   /* X[4] 待命区最下列 */
};

/* Y轴5个离散坐标（从左到右） */
const int32_t board_y_step[BOARD_Y_POINT_COUNT] = {
  2851,  /* Y[0] 左待命区 */
  4938,  /* Y[1] 博弈区左列*/
  6099,  /* Y[2] 博弈区中心列 */
  7229,  /* Y[3] 博弈区右列 */
  9399   /* Y[4] 右待命区 */
};

/* 19格位映射（cell_id -> x_idx, y_idx）
 * 注意：cell_id 0~18 对应用户编号 1~19
 *
 * 用户编号定义：
 * 1~5   : 右待命区（从下到上，即 X从下到上, Y=4） -> X[4]..X[0], Y[4]
 * 6~10  : 左待命区（从下到上，即 X从下到上, Y=0） -> X[4]..X[0], Y[0]
 * 11~19 : 中间博弈区（从右到左、从下到上，即 Y=3..1, X=3..1）
 *         11~13: 下行(X[3]), 右中左(Y[3], Y[2], Y[1])
 *         14~16: 中行(X[2]), 右中左(Y[3], Y[2], Y[1])
 *         17~19: 上行(X[1]), 右中左(Y[3], Y[2], Y[1])
 */
const BoardCellMap_t board_cell_map[BOARD_CELL_COUNT] = {
  /* 右待命区 5 格（编号1~5，从下到上：X[4]->X[0], Y[4]固定） */
  {4U, 4U}, {3U, 4U}, {2U, 4U}, {1U, 4U}, {0U, 4U},

  /* 左待命区 5 格（编号6~10，从下到上：X[4]->X[0], Y[0]固定） */
  {4U, 0U}, {3U, 0U}, {2U, 0U}, {1U, 0U}, {0U, 0U},

  /* 中间博弈区 9 格（编号11~19，从右到左、从下到上） */
  /* 下行 11~13 */
  {3U, 3U}, {3U, 2U}, {3U, 1U},
  /* 中行 14~16 */
  {2U, 3U}, {2U, 2U}, {2U, 1U},
  /* 上行 17~19 */
  {1U, 3U}, {1U, 2U}, {1U, 1U}
};

/* 
 * 通过格位编号 (0 ~ 18) 获取该格位的实际 X 和 Y 坐标步数。
 * 内部会自动调用前面定义的 board_x_step 和 board_y_step 数组。
 */
void BoardMap_GetCellStep(uint8_t cell_id, int32_t *x_step, int32_t *y_step)
{
  if (cell_id < BOARD_CELL_COUNT)
  {
    uint8_t x_idx = board_cell_map[cell_id].x_idx;
    uint8_t y_idx = board_cell_map[cell_id].y_idx;
    
    *x_step = board_x_step[x_idx];
    *y_step = board_y_step[y_idx];
  }
}

