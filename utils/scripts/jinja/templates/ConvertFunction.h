#pragma once

#include <etsi_its_{{ etsi_type }}_coding/{{ t_name }}.h>
#include <etsi_its_{{ etsi_type }}_msgs/{{ t_name }}.h>
{% for member in members -%}
#include <etsi_its_{{ etsi_type }}_conversion/convert{{ member.type }}.h>
{% endfor -%}

namespace etsi_its_{{ etsi_type }}_conversion {

void toRos_{{ t_name }}(const {{ t_name }}_t& in, etsi_its_{{ etsi_type }}_msgs::{{ t_name }}& out) {
{% for member in members -%}
{% if member.optional -%}
if(in.present == {{ t_name }}_PR::{{ t_name }}_PR_{{ member.type }}) {
toRos_{{ member.type }}(in.choice.{{ member.type }}, out.{{ member.type }});
out.choice = etsi_its_{{ etsi_type }}_msgs::{{ t_name }}::CHOICE_{{ member.type }};
}
{% endif -%}
{% endfor -%}
}