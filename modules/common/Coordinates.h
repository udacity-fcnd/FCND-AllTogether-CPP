#pragma once

#include "modules/math/V3F.h"

V3F global_to_local(const V3F& global_position, const V3F& global_home);

V3F local_to_global(const V3F& local_position, const V3F& global_home);