	.macro csr val
	csrr a0,\val
	.endm

# sscofpmf registers
	csr mhpmevent3h
	csr mhpmevent4h
	csr mhpmevent5h
	csr mhpmevent6h
	csr mhpmevent7h
	csr mhpmevent8h
	csr mhpmevent9h
	csr mhpmevent10h
	csr mhpmevent11h
	csr mhpmevent12h
	csr mhpmevent13h
	csr mhpmevent14h
	csr mhpmevent15h
	csr mhpmevent16h
	csr mhpmevent17h
	csr mhpmevent18h
	csr mhpmevent19h
	csr mhpmevent20h
	csr mhpmevent21h
	csr mhpmevent22h
	csr mhpmevent23h
	csr mhpmevent24h
	csr mhpmevent25h
	csr mhpmevent26h
	csr mhpmevent27h
	csr mhpmevent28h
	csr mhpmevent29h
	csr mhpmevent30h
	csr mhpmevent31h
	csr scountovf
