#!/usr/bin/env ruby

# files = `find src | egrep ".+\\.(h)$"`.split
# files.each { |name|

if ARGV.length < 1
  puts "Usage: in project root dir, #{__FILE__} file"
  exit
end

name = ARGV[0]

p name
guard = name.upcase.gsub(/[\/\\.]/, '_') + "_"
p guard

old = `grep ifndef #{name} | awk '{print $NF }'`.strip
p old
puts `sed -i 's/#{old}/#{guard}/g' #{name} | head -n 15`
