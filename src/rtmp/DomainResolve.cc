#include "DomainResolve.h"

#include <arpa/inet.h>
#include <inttypes.h>
#include <netdb.h>
#include <setjmp.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/types.h>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include "log/log.h"
static const int initGetHostByNameFail = 2939383;
static const int gethostbyname_timeout = 2939384;
static const int dns_gethostbyname_hostNotFound = 2939385;
static const int dns_gethostbyname_domainNotIpAddr = 2939386;
static const int refreshDomainCount = 2939387;
static const int refreshGetHostByNameFail = 2939388;

static sigjmp_buf jmpbuf;

static void alarm_handle(int) { siglongjmp(jmpbuf, 1); }

struct hostent *dns_gethostbyname_r(const char *hostname) {
  char buf[1024];
  int buflen = 1024;
  struct hostent hostbuf, *hp;
  int herr, hres;

  hres = gethostbyname_r(hostname, &hostbuf, buf, buflen, &hp, &herr);
  (void)hres;
  if (NULL == hp) {
    switch (herr) {
      case HOST_NOT_FOUND:
        // fprintf(stderr,"Host not found: %s\n",hostname);
        break;
      case NO_ADDRESS:
        // fprintf(stderr,"The requested name does not have an IP address:%s\n"
        // ,hostname);
        break;
      case NO_RECOVERY:
        // fprintf(stderr,"A non-recoverable name server error occurred while
        // resolving %s\n", hostname);
        break;
      case TRY_AGAIN:
        // fprintf(stderr,"A temporary error occurred on an authoritative name
        // server while resolving %s\n",hostname);
        break;
      default:
        // fprintf(stderr,"Unknown error code from gethostbyname_r for %s\n",
        // hostname);
        break;
    }
    return NULL;
  }

  return hp;
}

struct hostent *timeout_gethostbyname(const char *hostname, int timeout) {
  struct hostent *host = NULL;

  signal(SIGALRM, alarm_handle);
  if (sigsetjmp(jmpbuf, 1) != 0) {
    // fprintf(stderr,"gethostbyname timeout");
    return NULL;
  }

  struct itimerval value;
  value.it_value.tv_sec = timeout / 1000;
  value.it_value.tv_usec = (timeout % 1000) * 1000;
  value.it_interval.tv_sec = 0;
  value.it_interval.tv_usec = 0;
  setitimer(ITIMER_REAL, &value, NULL);

  // host = gethostbyname(hostname);
  host = dns_gethostbyname_r(hostname);
  // cancle timer
  value.it_value.tv_sec = 0;
  value.it_value.tv_usec = 0;
  value.it_interval.tv_sec = 0;
  value.it_interval.tv_usec = 0;
  setitimer(ITIMER_REAL, &value, NULL);

  return host;
}

int DomainResolve::init(uint32_t TimeoutMs, uint32_t IntervalMs) {
  // LOGSYS_WATER(_LC_ERROR_, "======== DomainResolve ========");

  m_TimeoutMs = TimeoutMs;
  m_IntervalMs = IntervalMs;

  // LOGSYS_WATER(_LC_ERROR_, "m_TimeoutMs:%u", m_TimeoutMs);
  // LOGSYS_WATER(_LC_ERROR_, "m_IntervalMs:%u", m_IntervalMs);

  return 0;
}

//域名解析失败，不启动，cmlb兜底只在定时刷新时使用
bool DomainResolve::addDomain(const std::string &domain) {
  if (domain.empty()) {
    return false;
  }

  struct hostent *host = timeout_gethostbyname(domain.c_str(), m_TimeoutMs);
  if (host == NULL) {
    return false;
  }

  sockaddr_in addr;
  addr.sin_addr = *((struct in_addr *)host->h_addr);
  domainMap[domain] = addr;

  // LOGSYS_WATER(_LC_ERROR_, "domain to ip: %s -> %s", domain.c_str(),
  // wbl::inet_ntoa_safe(addr.sin_addr).c_str());

  return true;
}

bool DomainResolve::getAddrByDomain(const std::string &domain,
                                    struct sockaddr_in &addr) {
  std::map<std::string, sockaddr_in>::const_iterator cit =
      domainMap.find(domain);
  if (cit == domainMap.end()) {
    return false;
  }

  addr = cit->second;
  return true;
}

void DomainResolve::refreshDomain(uint32_t now) {
  static unsigned int last = 0;
  if (last == 0 || last + m_IntervalMs <= now) {
    // Attr_API_Set(AttrId::DomainResolve::refreshDomainCount, 1);

    for (std::map<std::string, sockaddr_in>::iterator it = domainMap.begin();
         it != domainMap.end(); ++it) {
      struct hostent *host =
          timeout_gethostbyname(it->first.c_str(), m_TimeoutMs);
      if (host == NULL) {
        // 1);//定时拉取hostIP失败
        continue;
      }

      it->second.sin_addr = *((struct in_addr *)host->h_addr);
      tylog("%s refreshDomain: %s m_IntervalMs:%u %u", it->first.c_str(),
            inet_ntoa(it->second.sin_addr), m_IntervalMs, now - last);
    }

    last = now;
  }
}
