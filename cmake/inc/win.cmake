# Copyright 2019 Smarter Grid Solutions
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http ://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissionsand
# limitations under the License.

if (WIN32)
    message("PLATFORM: WINDOWS")
    set(PLATFORM_EXTENSION ".exe")
endif()

if(${CYGWIN})
    add_definitions(-D_WIN32_WINNT=_WIN32_WINNT_WIN7)
    add_definitions(-DNTDDI_VERSION=NTDDI_WIN7)
    add_definitions(-D__USE_W32_SOCKETS)
    add_definitions(-D_SCL_SECURE_NO_WARNINGS)
endif()
