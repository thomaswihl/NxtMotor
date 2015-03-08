import glob
import fileinput
import os

# Name of the executable
target = 'nxt'
# Your Atmel CPU
mcu = 'atmega88'
# Optimization level, choose from 0-3, s, g, ...
opt = '2'

env = DefaultEnvironment()
env.PrependENVPath('PATH', '/home/thomas/toolchain/avr8-gnu-toolchain-linux_x86_64/bin/')
env.Replace(CPPFLAGS = '-O' + opt + ' -std=c++11')
env.Replace(CC = 'avr-gcc -mmcu=' + mcu)
env.Replace(CXX = 'avr-g++ -mmcu=' + mcu)

sourceFiles = []
headerFiles = []
projectFilesFiles = glob.glob('*.files');
for file in projectFilesFiles:
    with open(file) as f:
        for line in f.readlines():
            # line = os.path.basename(line.strip())
            line = line.strip()
            if line.endswith('.cpp') or line.endswith('.c'):
                sourceFiles.append(line);
            elif line.endswith('.s') or line.endswith('.S'):
                sourceFiles.append(line);
            elif line.endswith('.hpp') or line.endswith('.h'):
                headerFiles.append(line);

includePaths = []
projectIncludesFiles = glob.glob('*.includes');
for file in projectIncludesFiles:
    with open(file) as f:
        for line in f.readlines():
            includePaths.append(os.path.basename(line.strip()))

defines = {}
projectDefines = glob.glob('*.config');
for file in projectDefines:
    with open(file) as f:
        for line in f.readlines():
            parts = line.strip().split()
            if len(parts) > 2 and parts[0] == '#define' and 'IGNORE' not in line:
                defines[parts[1]] = ' '.join(parts[2:])

envLeft = env.Clone()
envLeft['VARIANT'] = 'left'

envRight = env.Clone()
envRight['VARIANT'] = 'right'

envCenter = env.Clone()
envCenter['VARIANT'] = 'center'


Export('target sourceFiles includePaths defines')

env = envLeft
Export('env')
SConscript('SConscript', variant_dir=env['VARIANT'], src='src', duplicate=0)

env = envRight
Export('env')
SConscript('SConscript', variant_dir=env['VARIANT'], src='src', duplicate=0)

env = envCenter
Export('env')
SConscript('SConscript', variant_dir=env['VARIANT'], src='src', duplicate=0)
