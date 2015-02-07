import glob
import fileinput

# Name of the executable
target = 'nxt'
# Your Atmel CPU
mcu = 'atmega328p'
# Optimization level, choose from 0-3, s, g, ...
opt = '2'

env = DefaultEnvironment()
env.PrependENVPath('PATH', '/home/thomas/toolchain/avr8-gnu-toolchain-linux_x86_64/bin/')
env.Replace(CCFLAGS = '-O2 -D__AVR_ATmega328P__')
env.Replace(CC = 'avr-gcc')
env.Replace(CXX = 'avr-g++')

sourceFiles = []
headerFiles = []
projectFilesFiles = glob.glob('*.files');
for file in projectFilesFiles:
    with open(file) as f:
        for line in f.readlines():
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
            includePaths.append(line.strip())

defines = {}
projectDefines = glob.glob('*.config');
for file in projectDefines:
    with open(file) as f:
        for line in f.readlines():
            parts = line.strip().split()
            if len(parts) > 2 and parts[0] == '#define':
                defines[parts[1]] = ' '.join(parts[2:])

Program(target + '.elf', sourceFiles, CPPPATH = includePaths, CPPDEFINES=defines)
# Make hex
env.Command(target + ".hex", target + ".elf", 'avr-objcopy -R .eeprom -O ihex $SOURCE $TARGET')

# Show memory usage
env.Command(None, target + ".elf", "avr-size $SOURCE")
