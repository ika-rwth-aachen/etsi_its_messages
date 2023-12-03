- refactor converter.cpp udp2ros callback
- after changes for MAPEM headers, SPATEM headers look wrong?

````cpp
// convertAdvisorySpeed.h

// best try to handle RegionalExtension[]

// TODO: invalid use of incomplete type ‘struct RegionalExtension’
// AdvisorySpeed__regional->list is of type 'RegionalExtension' that is implemented nowhere
for (int i = 0; i < in.regional->list.count; i++) {
    spatem_msgs::RegionalExtension array;
    toRos_RegionalExtension(reinterpret_cast<RegionalExtension_364P0_t>(*(in.regional->list.array[i])), array);
    out.regional.push_back(array);
}
```


```cpp
// convertComputedLane.h

// TODO: nested CHOICE should be handled similar to e.g. HighFrequencyContainer
// will probably look like this
if (in.offsetXaxis.present == ComputedLane__offsetXaxis_PR::ComputedLane__offsetXaxis_PR_small) {
    toRos_DrivenLineOffsetSm(in.offsetXaxis.choice.small, out.offset_xaxis_small);
    out.offset_xaxis_choice = cam_msgs::HighFrequencyContainer::CHOICE_OFFSET_XAXIS_SMALL;
}
if (in.offsetXaxis.present == ComputedLane__offsetXaxis_PR::ComputedLane__offsetXaxis_PR_large) {
    toRos_DrivenLineOffsetLg(in.offsetXaxis.choice.large, out.offset_xaxis_large);
    out.offset_xaxis_choice = cam_msgs::HighFrequencyContainer::CHOICE_OFFSET_XAXIS_LARGE;
}

```