#ifndef MYGENERIC_H
#define MYGENERIC_H 1
 
#if !(defined(name2))
#  undef name2
#  define name2(a,b)      _name2_aux(a,b)
#  define _name2_aux(a,b)      a##b
#endif
 
#endif /* !MYGENERIC_H */