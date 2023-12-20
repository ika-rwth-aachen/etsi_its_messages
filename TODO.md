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
