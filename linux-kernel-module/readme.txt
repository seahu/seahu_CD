Linux driver for 1-wire slave device with family code 0xCD 
----------------------------------------------------------

This is direcroty with linux kernel driver for w1_seahu_cd (Multisensors 1-wire device based on Open-source android library)
Normal linux drive is part of linux source code and compile with whole linux kernel.
This driver is in format for  compile separately outside of linux source code.
For sucessfully compilatiom must frist insatll linux kernel headres.

Notes in Czech:
Protoze linux postupne jak se meni jeho verze, tak se postupne meni i 1-Wire api.
Takze jednotlive rady linuxu zde maji svuj adresar a ovldac pro danou 
verzi linuxu, dale pak script doit, ktery provadi kompilaci a prekopiravni jadra do spravneho mista.
