import pyperclip

s = """
		Mean diff. 	Std. Error 	t 	p 	95% CI lower limit 	95% CI upper limit
S1 Q12	S2 Q12	0,96	0,303	3,172	,062	0,34	1,58
S1 Q12	S3 Q12	0,12	0,211	0,569	1	-0,31	0,55
S1 Q12	S4 Q12	2,92	0,465	6,277	<.001	1,96	3,88
S1 Q12	S5 Q12	0,92	0,215	4,271	,004	0,48	1,36
S1 Q12	S6 Q12	2,32	0,315	7,371	<.001	1,67	2,97
S2 Q12	S3 Q12	-0,84	0,325	-2,585	,244	-1,51	-0,17
S2 Q12	S4 Q12	1,96	0,521	3,761	,014	0,88	3,04
S2 Q12	S5 Q12	-0,04	0,349	-0,115	1	-0,76	0,68
S2 Q12	S6 Q12	1,36	0,355	3,827	,012	0,63	2,09
S3 Q12	S4 Q12	2,8	0,526	5,323	<.001	1,71	3,89
S3 Q12	S5 Q12	0,8	0,216	3,703	,017	0,35	1,25
S3 Q12	S6 Q12	2,2	0,342	6,441	<.001	1,5	2,9
S4 Q12	S5 Q12	-2	0,507	-3,948	,009	-3,05	-0,95
S4 Q12	S6 Q12	-0,6	0,486	-1,233	1	-1,6	0,4
S5 Q12	S6 Q12	1,4	0,316	4,427	,003	0,75	2,05
"""

def convert2float(p):
    if p[0]==',':
        p = '0.' + p[1:]
        return float(p)
    if p=="<.001":
        return 0.0009
    else:
        return float(p.replace(',','.'))

results = ""
for l in s.splitlines()[2:]:
    p = l.split('\t')[5]
    p = convert2float(p)

    if p>0.05:
        results += "ns"
    elif p>0.01:
        results += "*"
    elif p>0.001:
        results += "**"
    elif p>0.0001:
        results += "***"
    else:
        results += "****"
    
    results += " "

print(results)
pyperclip.copy(results)
