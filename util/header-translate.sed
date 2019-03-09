# Run as 'sed -f $THIS_SCRIPT_NAME <c_header.h > zig_file.zig'

# Get rid of deprecated definitions, assumes only one #if...#endif block
/^#ifndef DEPRECATED/,/^#endif/d
# Get rid of include guard
/^#ifndef/,+1d
/^#endif/d

# Make backslash extended lines be on same line
/\\\s*$/ {
  N
  s/\\\s*\n//
}

# Get bit definitions
s/^#define\s\+\([a-zA-Z0-9_]\+\)\s\+\([a-zA-Z0-9_]\+\)\(.*\)/pub const \1 = usize(\2);\3/

# Get single address register definitions, discernable by definition as dereferenced pointer
s/^#define\s\+\([a-zA-Z0-9_]\+\)\s\+(\*((volatile uint32_t\s*\*)\(0x[0-9A-Fa-f]\+\)))\(.*\)/pub const \1 = @intToPtr(*volatile u32, \2);\3/

# Get multiple address spanning registers, discernable by definition as un-dereferenced pointer
s/^#define\s\+\([a-zA-Z0-9_]\+\)\s\+((volatile uint32_t\s*\*)\(0x[0-9A-Fa-f]\+\))\(.*\)/pub const \1 = @intToPtr([*]volatile u32, \2);\3/
