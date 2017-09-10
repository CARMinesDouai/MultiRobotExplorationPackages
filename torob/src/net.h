/*
 * <one line to give the library's name and an idea of what it does.>
 * Copyright (C) 2016  Guillaume L. <guillaume.lozenguez@mines-douai.fr>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef NETDATA_H
#define NETDATA_H

#include "data.h"

class NetData
{
protected:
   int a_port;
   std::string a_error;
   int a_server_sockfd;
   
public:
    NetData( int port, bool isServer );
    ~NetData();
    
    bool init_server();
    bool close_server();

    bool ok(int);
    bool you_ok(int);

    const std::string getError()const{ return a_error; }
    bool send( const std::string target, const mia::Data & d );
    bool wait( mia::Data & d );
};

#endif // NETDATA_H
