# Breadboard Testing Code for Memory Bank Controllers used in Nintendo® Game Boy® Cartridges

This is my personal testing code to collect information about Game Boy® MBCs and deduce how they work.
This code is for ATmega8515, because I had those lying around due to the Game Boy Cart Flasher project,
see https://github.com/Tauwasser/GBCartFlasher.

There is one branch per MBC and/or test setup.

## Introduction

All MBC chips are on break-out boards with 0.1" headers. They're directly connected to the relevant
ATmega8515 pins, as indicated in the comment at the top of `MBC_Test.c`.

All pin groups are then #defined for easy use in functions that are shared between branches.

The main tests are done in the mainloop, which uses a uart to perform individual tests.

## Compilation

Use Atmel Studio v6.1 or later to compile this firmware. You should have at least one .hex file after 
successful compilation.

## Remarks

Keep in mind the code is evolving, so I usually won't backport changes from *later* branches to 
*earlier* branches without a compelling reason to do so.

There's not a whole lot of documentation beyond what I personally require to kind of know what's
going on, so feel free to shoot me an email if you have questions.

## Licensing

This software (except `uart.{c,h}`) is licensed under the Creative Commons Attribution-ShareAlike 4.0
International License.

```
Copyright (C) 2015 Tauwasser (tauwasser@tauwasser.eu)

This work is licensed under the Creative Commons Attribution-ShareAlike 4.0
International License.

To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/
or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
```

`uart.{c,h}` were adapted from an old NXP LP21xx project I had lying around which used these UART
functions by *R O SoftWare*. All in all, just the function prototypes and some macros remain.
Their copyright notice is as follows.

```
Copyright 2004, R O SoftWare
No guarantees, warrantees, or promises, implied or otherwise.
May be used for hobby or commercial purposes provided copyright
notice remains intact.
```

## Legalese

I'm not affiliated with Nintendo in any way. I'm not affiliated with R O SoftWare either.
Nor with any other company whose chips I might look at using this software.

Game Boy® is a registered trademark by Nintendo. Nintendo® is a registered trademark.
All other trademarks are property of their respective owner.
