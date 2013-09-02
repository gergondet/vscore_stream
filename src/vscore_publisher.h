#ifndef _H_VSCOREPUBLISHER_H_
#define _H_VSCOREPUBLISHER_H_

#include <visionsystem/plugin.h>

struct VSCorePublisherImpl;

class VSCorePublisher : public visionsystem::Plugin
{
public:
    VSCorePublisher( visionsystem::VisionSystem* vs, std::string sandbox );

    ~VSCorePublisher();

    bool pre_fct();

    void preloop_fct();

    void loop_fct();

    bool post_fct();
private:
    VSCorePublisherImpl * impl;
};

PLUGIN(VSCorePublisher);

#endif // Include guard
