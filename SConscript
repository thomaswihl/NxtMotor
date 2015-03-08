Import('env target sourceFiles includePaths defines')

localTarget = target + '-' + env['VARIANT']

localDefines = defines.copy()
localDefines[env['VARIANT'].upper()] = 1
env.Program(localTarget + '.elf', sourceFiles, CPPPATH = includePaths, CPPDEFINES = localDefines)

# Make hex
env.Command(localTarget + '.hex', localTarget + '.elf', 'avr-objcopy -R .eeprom -O ihex $SOURCE $TARGET')

# Show memory usage
env.Command(None, localTarget + ".elf", "avr-size $SOURCE")

# Disassemble
env.Command(localTarget + ".S", localTarget + ".elf", 'avr-objdump -S $SOURCE > $TARGET')
