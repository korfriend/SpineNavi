#pragma once

#define SCANIMG_W 1296
#define SCANIMG_H 1296

#define SAFE_DELETE_OBJECT(p)	{ if(p) delete(p); p=nullptr; }
#define SAFE_DELETE_ARRAY(p)	{ if(p) delete[](p); p=nullptr; }

#define CFG_LISTEN_PORT 22222
#define CFG_SIZE_MAX_FD 1000
#define CFG_SIZE_MAX_RECV_BUFFER 5000