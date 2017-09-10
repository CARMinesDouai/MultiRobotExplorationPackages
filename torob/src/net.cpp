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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

// pour le client uniquement:
#include <netdb.h>

#include "net.h"

using namespace::mia;
using namespace::std;

#define BUFFER_SIZE 1014

void cout_flag( const mia::Data &d, int i, int size ){
  cout << "\tflag " << i << "-" << i+size << "/" << d.a_flag_size << ": " << d.a_flag[i]
    << ", " << d.a_flag[i+1] << ", " << d.a_flag[i+2]
    << "..." << d.a_flag[i+size/2]
    << "..." << d.a_flag[i+size-1] << endl;
}

void cout_value( const mia::Data &d, int i, int size ){
  cout << "\tvalue " << i << "-" << i+size << "/" << d.a_value_size << ": " << d.a_value[i]
    << ", " << d.a_value[i+1] << ", " << d.a_value[i+2]
    << "..." << d.a_value[i+size/2]
    << "..." << d.a_value[i+size-1] << endl;
}

NetData::NetData( int port, bool isServer ):
a_server_sockfd(-1),
a_port(port)
{
  a_error= "";
  if( isServer ) init_server();
}

bool NetData :: init_server()
{
  a_server_sockfd = socket(AF_INET, SOCK_STREAM, 0);// TCP socket
  if (a_server_sockfd < 0)
  {
    a_error= std::string("ERROR opening socket :") + strerror(errno);
    return false;
  }
  
  struct sockaddr_in serv_addr;
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(a_port);
  if (bind(a_server_sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
  {
    a_error= std::string("ERROR on binding :") + strerror(errno);
    return false;
  }
  
  listen(a_server_sockfd, 5);
  
  return true;
}

bool NetData :: close_server(){}

NetData :: ~NetData()
{
  if (a_server_sockfd < 0)
  {
    shutdown(a_server_sockfd, SHUT_RDWR);
    close(a_server_sockfd);
  }
}

bool NetData::ok(int sofd)
{
  if (write(sofd, "ok", 4) < 0)
  {
    a_error= "ERROR writing to socket";
    
    shutdown(sofd, SHUT_RDWR);
    close(sofd);
    return false;
  }
  return true;
}

bool NetData::you_ok(int sofd)
{
  char buffer[4];
  bzero(buffer,4);
  
  if ( read(sofd,buffer,4) < 0 )
  {
    a_error= "OK ERROR: reading from socket";
    return false;
  }
  
  if( buffer[0] == 'o' && buffer[1] == 'k' && buffer[2] == 0 )
    return true;
  
  a_error= "OK ERROR: not an ok message";
  return false;
}

bool NetData::send( const std::string target, const Data & d ) {
  
  cout << "send :" << endl;
  
  int sockfd;
  struct hostent *server;
  
  // Connection to server :
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
  {
    a_error= "SEND ERROR: opening socket";
    return false;
  }
  
  server = gethostbyname( target.c_str() );
  if (server == NULL)
  {
    a_error= "SEND ERROR: no such host";
    return false;
  }
  
  struct sockaddr_in serv_addr;
  
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy((char *)server->h_addr,
        (char *)&serv_addr.sin_addr.s_addr,
        server->h_length);
  serv_addr.sin_port = htons(a_port);
  if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
  {
    a_error= "SEND ERROR: connecting";
    return false;
  }
  
  // Send meta_data :
  
  int meta[3];
  meta[0]= d.mesage_size();
  meta[1]= d.flag_size();
  meta[2]= d.value_size();
  
  cout << "\tmeta: " << meta[0] << ", " << meta[1] << ", " << meta[2] << endl;
  
  if (write( sockfd, meta, 3*sizeof(int) ) < 0)
  {
    a_error= "SEND ERROR: writing to socket";
    return false;
  }
  
  // Send msg :
  for( int i(0); i < d.a_mesage_size; i+= BUFFER_SIZE)
  {
    int size= min(BUFFER_SIZE, d.a_mesage_size-i);
    
    if (write( sockfd, &(d.a_mesage[i]), size ) < 0)
    {
      a_error= "SEND ERROR: writing to socket";
      return false;
    }
    
    if( !you_ok(sockfd) )
      return false;
  }
  
  const int sizeo4= BUFFER_SIZE/4;
  
  // Send flag :
  for( int i(0); i < d.a_flag_size; i+= sizeo4)
  {
    int size= min(sizeo4, d.a_flag_size-i);
    if (write( sockfd, &(d.a_flag[i]), size*sizeof(int) ) < 0)
    {
      a_error= "SEND ERROR: writing to socket";
      return false;
    }
    
    cout_flag(d, i, size);
    
    if( !you_ok(sockfd) )
      return false;
  }
  
  // Send value :
  for( int i(0); i < d.a_value_size; i+= sizeo4)
  {
    int size= min(sizeo4, d.a_value_size-i);
    if (write( sockfd, &(d.a_value[i]), size*sizeof(float) ) < 0)
    {
      a_error= "SEND ERROR: writing to socket";
      return false;
    }
    
    cout_value(d, i, size);
    
    if( !you_ok(sockfd) )
      return false;
  }
  
  if( !you_ok(sockfd) )
    return false;
  
  shutdown(sockfd, SHUT_RDWR);
  
  a_error= "";
  return true;
}

bool NetData::wait(Data & d)
{
  cout << "wait :" << endl;
  
  int newsockfd;
  socklen_t clilen;
  struct sockaddr_in cli_addr;
  
  
  clilen = sizeof(cli_addr);
  newsockfd = accept(a_server_sockfd, (struct sockaddr *) &cli_addr, &clilen);
  if (newsockfd < 0)
  {
    a_error= std::string("ERROR on accept") + strerror(errno);
    
    shutdown(newsockfd, SHUT_RDWR);
    close(newsockfd);
    return false;
  }
  
  // read meta :
  int meta[3];
  if ( read( newsockfd, meta, 3*sizeof(int) ) < 0)
  {
    a_error= "ERROR reading meta from socket";
    
    shutdown(newsockfd, SHUT_RDWR);
    close(newsockfd);
    return false;
  }
  
  cout << "\tmeta: " << meta[0] << ", " << meta[1] << ", " << meta[2] << endl;
  
  // Initialize data d
  d.initialize( meta[0], meta[1], meta[2] );
  
  // read msg :
  bzero( d.a_mesage,  d.a_mesage_size+1 );
  for( int i(0); i < d.a_mesage_size; i+= BUFFER_SIZE)
  {
    int size= min(BUFFER_SIZE, d.a_mesage_size-i);
    if ( read(newsockfd, &(d.a_mesage[i]), size) < 0)
    {
      a_error= "ERROR reading data from socket";
      
      shutdown(newsockfd, SHUT_RDWR);
      close(newsockfd);
      return false;
    }
    
    if( !ok(newsockfd) )
    {
      shutdown(newsockfd, SHUT_RDWR);
      close(newsockfd);
      return false;
    }
  }
  
  const int sizeo4= BUFFER_SIZE/4;
  
  // read flag :
  for( int i(0); i < d.a_flag_size; i+= sizeo4)
  {
    int size= min(sizeo4, d.a_flag_size-i);
    if ( read( newsockfd, &(d.a_flag[i]), size*sizeof(int) ) < 0)
    {
      a_error= "ERROR reading data from socket";
      
      shutdown(newsockfd, SHUT_RDWR);
      close(newsockfd);
      return false;
    }
    
    cout_flag(d, i, size);
    
    if( !ok(newsockfd) )
    {
      shutdown(newsockfd, SHUT_RDWR);
      close(newsockfd);
      return false;
    }
  }
  
  // read values :
  for( int i(0); i < d.a_value_size; i+= sizeo4)
  {
    int size= min(sizeo4, d.a_value_size-i);
    
    if (read( newsockfd, &(d.a_value[i]), size*sizeof(float) ) < 0)
    {
      a_error= "ERROR reading data from socket";
      
      shutdown(newsockfd, SHUT_RDWR);
      close(newsockfd);
      return false;
    }
    
    cout_value(d, i, size);
    
    if( !ok(newsockfd) )
    {
      shutdown(newsockfd, SHUT_RDWR);
      close(newsockfd);
      return false;
    }
  }
  
  // Send data :
  if( !ok(newsockfd) )
  {
    shutdown(newsockfd, SHUT_RDWR);
    close(newsockfd);
    return false;
  }
  
  shutdown(newsockfd, SHUT_RDWR);
  close(newsockfd);
  
  a_error= "";
  
  return true;
}
