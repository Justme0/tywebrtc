#pragma once

void H264ParseSps(const char *sps, unsigned int sps_size, int &width,
                  int &height);
