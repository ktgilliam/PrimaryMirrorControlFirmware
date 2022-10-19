/*******************************************************************************
Copyright 2022
Steward Observatory Engineering & Technical Services, University of Arizona
This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <https://www.gnu.org/licenses/>.
*******************************************************************************/

/**
@brief 
@author Nestor Garcia
@date October 17, 2022
@file primary_mirror_network.h

*/

#ifndef PRIMARY_MIRROR_NETWORK_H
#define PRIMARY_MIRROR_NETWORK_H

bool networkInit();
void read_command();
void write_data();
void messageParser();

#endif // PRIMARY_MIRROR_NETWORK_H/*