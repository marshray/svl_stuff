// vim: et ts=63 sw=4 tw=100 fileencoding=utf-8 ff=unix
//
// Copyright 2019 Marsh Ray
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

`default_nettype none

//-------------------------------------------------------------------------------------------------|

// Some 3rd grade math functions not supplied by programming environment.

`define floor_ti(r) (time'(r) <= (r) ? time'(r) : time'(r) - 1)
`define ceil_ti(r)  ((r) <= time'(r) ? time'(r) : time'(r) + 1)

`define floor_li(r) (longint'(r) <= (r) ? longint'(r) : longint'(r) - 1)
`define ceil_li(r)  ((r) <= longint'(r) ? longint'(r) : longint'(r) + 1)

`define min(l, r) ((r) < (l) ? (r) : (l))
`define max(l, r) ((l) < (r) ? (r) : (l))

//-------------------------------------------------------------------------------------------------|
