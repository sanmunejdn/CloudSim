#include "IPlcCommClient.h"

#include "PlcCommClientImpl.h"

std::unique_ptr<IPlcCommClient> createPlcCommClient()
{
    return std::make_unique<PlcCommClientImpl>();
}
