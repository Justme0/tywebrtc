#!/usr/bin/env ruby

files = `find src | egrep ".+\\.(h)$"`.split

files.each { |name|
  p name
  guard = name.upcase.gsub(/[\/\\.]/, '_') + "_"
  p guard

  old = `grep ifndef #{name} | awk '{print $NF }'`.strip
  p old
  puts `sed -i 's/#{old}/#{guard}/g' #{name} | head -n 15`

}
