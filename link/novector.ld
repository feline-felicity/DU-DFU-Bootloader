SECTIONS{
    /DISCARD/ : {
        /* Discard everything from CRT, including default vector tables */
        *crt*.o(*)
    }
}
INSERT BEFORE .text
