#
# Assume arm all the way up and bucket out
#
curl -X POST http://localhost:5000/bucket/down/200/2000
curl -X POST http://localhost:5000/bucket/down/200/2000
curl -X POST http://localhost:5000/bucket/up/200/2000
curl -X POST http://localhost:5000/bucket/down/200/2000
curl -X POST http://localhost:5000/bucket/up/200/2000
curl -X POST http://localhost:5000/bucket/down/200/2000
curl -X POST http://localhost:5000/bucket/up/200/2000

#http://localhost:5000/mainarm/down/100/1000
#http://localhost:5000/mainarm/up/100/1000
#http://localhost:5000/rotate/right/50/1000
#http://localhost:5000/rotate/left/50/1000
#http://localhost:5000/smallarm/up/50/1000
#http://localhost:5000/smallarm/down/50/1000
#http://localhost:5000/bucket/down/50/1000
#http://localhost:5000/bucket/up/50/1000
#http://localhost:5000/move/backward/200/1000
#http://localhost:5000/move/forward/200/1000
#http://localhost:5000/turn/left/200/1000
#http://localhost:5000/turn/right/200/1000
