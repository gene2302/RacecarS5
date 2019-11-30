#!/usr/bin/env python

import socket
from struct import *

HOST = '127.0.0.1'
PORT = 65432
format = "!Ixxxxxxxxxxxx"
data = ""

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
while True:
    input = raw_input("Selectionner l'option voulue\n1-RPOS\n2-OBSF\n3-RBID\n4-Exit\n")
    try :
	command = int(input)
   	format = "!Ixxxxxxxxxxxx"
    	if command == 1:
        	s.send(pack(format,command))
        	format = "!fffxxxx"
        	data = unpack(format,s.recv(1024))
    	elif command == 2:
        	s.send(pack(format,command))
        	format = "!Ixxxxxxxxxxxx"
        	data = unpack(format,s.recv(1024))[0]                
    	elif command == 3:
        	s.send(pack(format,command))
        	format = "!Ixxxxxxxxxxxx"
        	data = unpack(format,s.recv(1024))[0]
	elif command ==4:
		break  
    	else:
		print("La saisie ne correspond a aucune des options suivantes.") 
    except ValueError:
	print("Veuillez entrer une valeur numerique.")
    print data
s.close()

