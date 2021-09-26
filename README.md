# Sequence
Block based sequence description framework.

## Demo codes
### Print&Delay
```
#include "Sequence/inc/sequence.h"
#include <windows.h>

using namespace std;
using namespace seq;

int main()
{
    Sequence sequence
    (
        "Main",
        new block::Print("Hello"),
        new block::Delay(1.0),
        new block::Print("World"),
        new block::Delay(1.0)
        );

    sequence.compile(true);
    sequence.start();

    while (1) //use timer callback instead
    {
        Sequence::spinOnce();
        Sleep(1);
    }
}


```
result:
```
Sequence started(Main)
|___Print(Hello)
Hello
|___Delay(1.000000)
|___Print(World)
World
|___Delay(1.000000)
Sequence terminated(Main)
```
