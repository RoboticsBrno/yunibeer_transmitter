from time import strftime as time
f = open('../version_info.hpp', 'r')
last = f.read()
f.close()
build = int(last[32:last.find(':', 32)]) + 1
build_info = 'build {0}: {1}'.format(build, time('%H:%M:%S %d.%m.%Y'))
f = open('../version_info.hpp', 'w')
f.write('const char* build_info = "{}";\n'.format(build_info));
f.close()
print build_info
