#!/usr/bin/env ruby

text = %q(// Copyright (c) 2024 The tywebrtc project authors. All Rights Reserved.
//
// Use of this source code is governed by a MIT license
// that can be found in the LICENSE file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS.  All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.
)

p text


files = `find src | egrep ".+\\.(h|cc)$"`.split

files.each { |name|
  p name
  match = `grep '2024 The tywebrtc project authors'  #{name}`
  if match != ''
    p match
    next
  end

  `echo -e "#{text}" | cat - #{name} > temp && mv temp #{name}`

#   break

}
