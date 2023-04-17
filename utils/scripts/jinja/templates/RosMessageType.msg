# ------------------------------------------------------------------------------
{% for line in asn1_definition.split('\n') -%}
# {{ line }}
{% endfor -%}
# ------------------------------------------------------------------------------

{% for member in members -%}
{% for comment in member.comments -%}
# {{ comment }}
{% endfor -%}
{{ member.type }} {{ member.name }}
{% if member.optional -%}
bool {{ member.name }}_isPresent
{% endif -%}
{% for const in member.constants -%}
{{ const.type }} {{ const.name }} = {{ const.value }}
{% endfor %}
{% endfor %}