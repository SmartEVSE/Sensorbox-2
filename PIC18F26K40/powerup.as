#include <xc.inc>
	GLOBAL powerup, start
	PSECT powerup, class=CODE, delta=1, reloc=2
powerup:
	BSF NVMCON1, 7
	GOTO start
	end