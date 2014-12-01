env = DefaultEnvironment(CXX = 'avr-g++',
                   CC = 'avr-gcc', CCFLAGS = '-O2 -D__AVR_ATmega328P__')
env['ENV']['PATH'] += ':/home/thomas/toolchain/avr8-gnu-toolchain-linux_x86_64/bin'
print(env['ENV'])
Program('nxtmotor', Glob('*.cpp'), CPPPATH = '.')